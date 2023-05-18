cmake_minimum_required(VERSION 3.8)
project(motion_planning_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)  

add_executable(cartesian_space_trajectory src/cartesian_space_trajectory.cpp)
target_include_directories(cartesian_space_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cartesian_space_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cartesian_space_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
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
  "moveit_visual_tools"
)

add_executable(linear_cartesian_space_trajectory src/linear_cartesian_space_trajectory.cpp)
target_include_directories(linear_cartesian_space_trajectory PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(linear_cartesian_space_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  linear_cartesian_space_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)


install(TARGETS cartesian_space_trajectory joint_space_trajectory linear_cartesian_space_trajectory
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

ament_package()
