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
git clone https://github.com/Krawus/motion_planning_interfaces.git
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

| ID | Planner                  |
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


To plan trajectory you have call /plan_trajectory_service service using motion_planning_interfaces/srv/PlanTrajectory interface.

The structure of the interface:

```
float64[6] joints
float64[7] pose
uint8 type
uint8 planner
---
bool executed
```

This interface allows you to select the space of the planned trajectory (joint space, cartesian space, linear cartesian trajectory) by selecting its ID, setting goal for manipulator:
  * in joint space user can set the configuration of every joint of manipulator (**in degrees**), 
  * in both cartesian and linear cartesian user can set the cartesian position (x,y,z) and the orientation quaternion (w,x,y,z) of the destination position  

While using the joint space you should enter the '*joints*' parameter, and when using cartesian space you should enter the '*pose*' parameter.
The '*type*' and '*planner*' parameters are used in both cases.
Type parameter is used to define which type of trajectory is used. Planner parameter is responsible for the planner selected by the user.
Available trajectory types:

| Type | Trajectory Type                 |
|----|--------------------------|
| 0  | Joints Space        |
| 1  | Cartesian Space        |
| 2  | Linear Cartesian Space   |

\* joint space call\*
```
ros2 service call /plan_trajectory_service motion_planning_interfaces/srv/PlanTrajectory "{joints: [65,-40,35,20,-22,0.0],type: 0, planner: 1}"
```

\* cartesian space call\*
```
ros2 service call /plan_trajectory_service motion_planning_interfaces/srv/PlanTrajectory "{pose: [1,1,1,0,0,0,0.0],type: 1, planner: 1}"
```

\* linear cartesian space call \*
```
ros2 service call /plan_trajectory_service motion_planning_interfaces/srv/PlanTrajectory "{pose: [0.5,1,1,0.0,0.0,0.0,0.0],type: 2, planner: 0}"
```


The motion_planning_server also allows you to add and delete objects to be avoided by the manipulator.  

To add object you need to make call to the /plan_trajectory_service using motion_planning_interfaces/srv/ObstacleManagement.

```
float64[3] box_position
float64[3] box_dimensions
string box_id
string type
---
bool created
```

To add new object to the planning scene user have to enter the '*box_position*' parameter that is the starting point in cartesian space (x,y,z) and the box_dimensions that is box size in each axis (x,y,z). You can also specify the '*box_id*' parameter, if not specified, the service assumes the next box number *x*, and the id is equal to '*box_**x***'. You need to enter the '*type*' (add) parameter, which defines the type of the operation.

\* add object call \*
```
ros2 service call /obstacles_service motion_planning_interfaces/srv/ObstacleManagement "{box_position:[0.5,0.3,0.5], box_dimensions:[0.3,0.3,0.3], box_id: 'box_a',type: 'add'}"
```

\* delete object call \*
```
ros2 service call /obstacles_service motion_planning_interfaces/srv/ObstacleManagement "{box_id: 'box_a',type: 'remove'}"
```

To remove the object from the planning scene the only parameters you need to enter are '*type*' (remove) and '*box_id*' of the box you are going to remove.





## Examples of package functionalities
### Planning trajectory in joint space

<p align="center">
    <img width="63%" src="https://i.imgur.com/Bx2Rfpg.gif">
</p>

### Planning trajectory in cartesian space
<p align="center">
    <img width="63%" src="https://i.imgur.com/q9cueKA.gif">
</p>

### Planning linear cartesian trajectory
<p align="center">
    <img width="63%" src="https://i.imgur.com/MB4ayzO.gif">
</p>

### Adding object and avoiding it, then deleting object
<p align="center">
    <img width="63%" src="https://i.imgur.com/SP25fZu.gif">
</p>
