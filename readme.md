## About
This is a ROS2 package for planning UR5e trajectories in joint space, cartesian space and to plan linear cartesian trajectories. The node also has functionality to select the planner and to add and remove objects from the scene that will be avoided when planning the robot's path.

## Installation  
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
Install motion_planning_moveit_ur.  
```
git clone https://github.com/Krawus/motion_planning_moveit_ur.git
```

\
Download motion planning interfaces.  
```
git clone git@github.com:Krawus/motion_planning_interfaces.git
```

## Usage  
\
Start Universal Robots Simulator with UR5e in browser. (If you are running ROS2 in docker you have to run this bash script native).  
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

Launch motion planning node service
```
ros2 launch motion_planning_moveit motion_planning_server.launch.py 
```  
\
Now you can plan trajectories in joint space, cartesian space and plan linear cartesian trajectory. Using this service you are also able to choose planner for your trajectory by selecting relevant id. List of planners below:  

| ID | planner                  |
|----|--------------------------|
| 0  | SBLkConfigDefault        |
| 1  | ESTkConfigDefault        |
| 2  | LBKPIECEkConfigDefault   |
| 3  | BKPIECEkConfigDefault    |
| 4  | KPIECEkConfigDefault     |
| 5  | RRTkConfigDefault        |
| 6  | RRTConnectkConfigDefault |
| 7  | RRTstarkConfigDefault    |
| 8  | TRRTkConfigDefault       |
| 9  | PRMkConfigDefault        |
| 10 | PRMstarkConfigDefault    |  


To plan trajectory you have call /joint_space_trajectory_service service using motion_planning_interfaces/srv/JointTrajectory interface.

\*Structure of the message \*

This interface allows you to select the space of the planned trajectory (joint space, cartesian space, linear cartesian trajectory) by selecting its ID, setting goal for manipulator:
  * in joint space user can set the configuration of every joint of manipulator (**in degrees**), 
  * cartesian ...,  
  * in linear cartesian ...,

\* example call \*

The motion_planning_server also allows you to add and delete objects to be avoided by the manipulator.  

To add object you need to make call to the /joint_space_trajectory_service using \* INSERT INTERFACE NAME  \*  



## Examples of package functionalities
### Planning trajectory in joint space

<p align="center">
    <img width="63%" src="https://i.imgur.com/MQ7DeJk.gif">
</p>
