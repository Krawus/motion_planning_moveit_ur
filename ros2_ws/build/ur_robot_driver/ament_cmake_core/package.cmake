set(_AMENT_PACKAGE_NAME "ur_robot_driver")
set(ur_robot_driver_VERSION "2.2.6")
set(ur_robot_driver_MAINTAINER "Denis Stogl <denis@stoglrobotics.de>, Felix Exner <exner@fzi.de>, Robert Wilbrandt <wilbrandt@fzi.de>")
set(ur_robot_driver_BUILD_DEPENDS "controller_manager" "controller_manager_msgs" "geometry_msgs" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "rclpy" "std_msgs" "std_srvs" "tf2_geometry_msgs" "ur_client_library" "ur_controllers" "ur_dashboard_msgs" "ur_description" "ur_msgs")
set(ur_robot_driver_BUILDTOOL_DEPENDS "ament_cmake" "ament_cmake_python")
set(ur_robot_driver_BUILD_EXPORT_DEPENDS "controller_manager" "controller_manager_msgs" "geometry_msgs" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "rclpy" "std_msgs" "std_srvs" "tf2_geometry_msgs" "ur_client_library" "ur_controllers" "ur_dashboard_msgs" "ur_description" "ur_msgs")
set(ur_robot_driver_BUILDTOOL_EXPORT_DEPENDS )
set(ur_robot_driver_EXEC_DEPENDS "force_torque_sensor_broadcaster" "joint_state_broadcaster" "joint_state_publisher" "joint_trajectory_controller" "launch" "launch_ros" "position_controllers" "robot_state_publisher" "ros2_controllers_test_nodes" "rviz2" "socat" "urdf" "velocity_controllers" "xacro" "controller_manager" "controller_manager_msgs" "geometry_msgs" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "rclpy" "std_msgs" "std_srvs" "tf2_geometry_msgs" "ur_client_library" "ur_controllers" "ur_dashboard_msgs" "ur_description" "ur_msgs")
set(ur_robot_driver_TEST_DEPENDS "docker.io" "launch_testing_ament_cmake")
set(ur_robot_driver_GROUP_DEPENDS )
set(ur_robot_driver_MEMBER_OF_GROUPS )
set(ur_robot_driver_DEPRECATED "")
set(ur_robot_driver_EXPORT_TAGS)
list(APPEND ur_robot_driver_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
