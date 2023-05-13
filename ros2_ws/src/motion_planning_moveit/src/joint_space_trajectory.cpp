#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <math.h>

// #define M_PI  3.14159265358979323846

static const rclcpp::Logger logger = rclcpp::get_logger("joint_traj_logger");

void setJointGroupPositionsInDegrees(std::vector<double> &jointConfiguration, double joint0, double joint1, double joint2, 
                                                    double joint3, double joint4, double joint5){
  jointConfiguration[0] = joint0 * M_PI/180.0;
  jointConfiguration[1] = joint1 * M_PI/180.0;
  jointConfiguration[2] = joint2 * M_PI/180.0;
  jointConfiguration[3] = joint3 * M_PI/180.0;
  jointConfiguration[4] = joint4 * M_PI/180.0;
  jointConfiguration[5] = joint5 * M_PI/180.0;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions nodeOptions;
  nodeOptions.automatically_declare_parameters_from_overrides(true);
  auto moveGroupNode = rclcpp::Node::make_shared("joint_space_trajectory", nodeOptions);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(moveGroupNode);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface moveGroup(moveGroupNode, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  const moveit::core::JointModelGroup* jointModelGroup = moveGroup.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::core::RobotStatePtr current_state = moveGroup.getCurrentState(10);

  std::vector<double> jointGroupPositions;
  current_state->copyJointGroupPositions(jointModelGroup, jointGroupPositions);

  setJointGroupPositionsInDegrees(jointGroupPositions, 50.0, -75.0, 0.0, 0.0, 30.0 ,0.0);

  moveGroup.setMaxVelocityScalingFactor(0.70);
  moveGroup.setMaxAccelerationScalingFactor(0.70);

  bool within_bounds = moveGroup.setJointValueTarget(jointGroupPositions);
  if (!within_bounds){
    RCLCPP_WARN(logger, "Position out of bounds!!!");
  }
  

  auto const [success, plan] = [&moveGroup]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(moveGroup.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    moveGroup.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  rclcpp::shutdown();



  return 0;
}