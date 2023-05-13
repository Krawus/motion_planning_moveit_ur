#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// Define planning group
static const std::string PLANNING_GROUP = "ur_manipulator";


int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "linear_cartesian_space_traj",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("linear_cartesian_space_traj");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Create cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Getting current pose
  geometry_msgs::msg::Pose currentPose = move_group_interface.getCurrentPose().pose;
//   waypoints.push_back(currentPose);

  // Setting destination pose 
  geometry_msgs::msg::Pose destinationPose = currentPose;
  destinationPose.position.z +=0.4;
  waypoints.push_back(destinationPose);
  
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
  RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  bool success = true; // temporary
//   plan.trajectory_ = trajectory;

//   // Create a plan to that target pose
//   auto const [success, plan] = [&move_group_interface]
//   {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(trajectory);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}