#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <math.h>

// Create a ROS logger
static const rclcpp::Logger logger = rclcpp::get_logger("joint_traj_logger");

// Define planning group
static const std::string PLANNING_GROUP = "ur_manipulator";

// Function is setting desired joint angles in degrees to manipulator's joint 
// configuration and calculating them into radians
 void setJointGroupPositionsInDegrees(std::vector<double> &jointConfiguration, double joint0, double joint1, double joint2, 
                                                    double joint3, double joint4, double joint5){
  jointConfiguration[0] = joint0 * M_PI/180.0;
  jointConfiguration[1] = joint1 * M_PI/180.0;
  jointConfiguration[2] = joint2 * M_PI/180.0;
  jointConfiguration[3] = joint3 * M_PI/180.0;
  jointConfiguration[4] = joint4 * M_PI/180.0;
  jointConfiguration[5] = joint5 * M_PI/180.0;
}

// Available planners
enum planner{
    SBLkConfigDefault,
    ESTkConfigDefault,
    LBKPIECEkConfigDefault,
    BKPIECEkConfigDefault,
    KPIECEkConfigDefault,
    RRTkConfigDefault,
    RRTConnectkConfigDefault,
    RRTstarkConfigDefault,
    TRRTkConfigDefault,
    PRMkConfigDefault,
    PRMstarkConfigDefault,
};


// Function is setting planner passed with params to move group
void setPlanner(moveit::planning_interface::MoveGroupInterface &moveGroup, const planner &p){

  switch (p){
    case SBLkConfigDefault: moveGroup.setPlannerId("SBLkConfigDefault"); break;
    case ESTkConfigDefault: moveGroup.setPlannerId("ESTkConfigDefault"); break;
    case LBKPIECEkConfigDefault: moveGroup.setPlannerId("LBKPIECEkConfigDefault"); break;
    case BKPIECEkConfigDefault: moveGroup.setPlannerId("BKPIECEkConfigDefault"); break;
    case KPIECEkConfigDefault: moveGroup.setPlannerId("KPIECEkConfigDefault"); break;
    case RRTkConfigDefault: moveGroup.setPlannerId("RRTkConfigDefault"); break;
    case RRTConnectkConfigDefault: moveGroup.setPlannerId("RRTConnectkConfigDefault"); break;
    case RRTstarkConfigDefault: moveGroup.setPlannerId("RRTstarkConfigDefault"); break;
    case TRRTkConfigDefault: moveGroup.setPlannerId("TRRTkConfigDefault"); break;
    case PRMkConfigDefault: moveGroup.setPlannerId("PRMkConfigDefault"); break;
    case PRMstarkConfigDefault: moveGroup.setPlannerId("PRMstarkConfigDefault"); break;
    default: moveGroup.setPlannerId("PRMkConfigDefault");
  }
}


int main(int argc, char * argv[])
{

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions nodeOptions;
  nodeOptions.automatically_declare_parameters_from_overrides(true);
  auto moveGroupNode = rclcpp::Node::make_shared("joint_space_trajectory", nodeOptions);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(moveGroupNode);
  std::thread([&executor]() { executor.spin(); }).detach();


  moveit::planning_interface::MoveGroupInterface moveGroup(moveGroupNode, PLANNING_GROUP);
  
  setPlanner(moveGroup, planner::ESTkConfigDefault);
  
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