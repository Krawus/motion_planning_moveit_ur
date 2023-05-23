Install ROS2 Universal Robots Driver
```
cd ros2_ws
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
\
Start Universal Robots Simulator with UR5e in browser
```
bash start_ursim.sh -m ur5e
```
\
Start controller
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```
\
Start MoveIt
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_fake_hardware:=true
```


download motion planning interfaces
```
git clone git@github.com:Krawus/motion_planning_interfaces.git
```

Launch motion planning node service
```
ros2 launch motion_planning_moveit motion_planning_server.launch.py 
```
Execute trajectory in joint space
```
ros2 service call /joint_space_trajectory_service motion_planning_interfaces/srv/JointTrajectory "{joints: [50,-75,0,0,30,0], planner: 
1}"
```

*insert lista plannerow i wartości tutaj albo da sie jako logger przy odpalaniu servera jako INFO z LOGGERA*