#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

// Define planning group
static const std::string PLANNING_GROUP = "ur_manipulator";


int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "linear_cartesian_space_traj",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
      


  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();   


visual_tools.deleteAllMarkers();
visual_tools.loadRemoteControl(); 
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "wrist_3_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);








  // Create a ROS logger
  auto const logger = rclcpp::get_logger("linear_cartesian_space_traj");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Create cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Getting current pose
  geometry_msgs::msg::Pose currentPose = move_group_interface.getCurrentPose("wrist_3_link").pose;
//   waypoints.push_back(currentPose);

  // Setting destination pose 
  geometry_msgs::msg::Pose destinationPose = currentPose;
  destinationPose.position.x=0.5;
  destinationPose.position.y=0.7;
  destinationPose.position.z=0.5;
  waypoints.push_back(destinationPose);
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
  RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  bool success = true; // temporary
  plan.trajectory_ = trajectory;
  moveit_visual_tools.publishAxisLabeled(currentPose, "start_pose");
  moveit_visual_tools.publishAxisLabeled(destinationPose, "target_pose");
  moveit_visual_tools.publishTrajectoryLine(trajectory, joint_model_group_ptr);
  moveit_visual_tools.trigger();


  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}