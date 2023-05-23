cmake_minimum_required(VERSION 3.8)
project(motion_planning_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motion_planning_interfaces REQUIRED)

add_executable(motion_planning_moveit src/motion_planning_moveit.cpp)
target_include_directories(motion_planning_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_planning_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  motion_planning_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(joint_space_trajectory src/joint_space_trajectory.cpp)
target_include_directories(joint_space_trajectory PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(joint_space_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joint_space_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(motion_planning_server src/motion_planning_server.cpp)
ament_target_dependencies(motion_planning_server rclcpp moveit_ros_planning_interface motion_planning_interfaces)


install(TARGETS motion_planning_moveit joint_space_trajectory motion_planning_server
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------
# find_package(rosidl_default_generators REQUIRED)

# set(srv_files
#   "srv/JointTrajectory.srv"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${srv_files}
# )

# ament_export_dependencies(rosidl_default_runtime)
#-----------------------------------------------

ament_package()
