#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <math.h>


#include "motion_planning_interfaces/srv/joint_trajectory.hpp"


geometry_msgs::msg::Pose createPoseMsg(const motion_planning_interfaces::srv::JointTrajectory::Request::SharedPtr request){
    geometry_msgs::msg::Pose msg;
    // Point
    msg.position.x = request->pose[0];
    msg.position.x = request->pose[1];
    msg.position.x = request->pose[2];
    // Orientation
    msg.orientation.w = request->pose[3];
    msg.orientation.x = request->pose[4];
    msg.orientation.y = request->pose[5];
    msg.orientation.z = request->pose[6];
    return msg;
}

void setJointGroupPositionsInDegrees(std::vector<double> &jointConfiguration, double joint0, double joint1, double joint2, 
                                                    double joint3, double joint4, double joint5){
  jointConfiguration[0] = joint0 * M_PI/180.0;
  jointConfiguration[1] = joint1 * M_PI/180.0;
  jointConfiguration[2] = joint2 * M_PI/180.0;
  jointConfiguration[3] = joint3 * M_PI/180.0;
  jointConfiguration[4] = joint4 * M_PI/180.0;
  jointConfiguration[5] = joint5 * M_PI/180.0;
}

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

void setPlanner(moveit::planning_interface::MoveGroupInterface &moveGroup, uint8_t p){

  switch (p){
    case 0: moveGroup.setPlannerId("SBLkConfigDefault"); break;
    case 1: moveGroup.setPlannerId("ESTkConfigDefault"); break;
    case 2: moveGroup.setPlannerId("LBKPIECEkConfigDefault"); break;
    case 3: moveGroup.setPlannerId("BKPIECEkConfigDefault"); break;
    case 4: moveGroup.setPlannerId("KPIECEkConfigDefault"); break;
    case 5: moveGroup.setPlannerId("RRTkConfigDefault"); break;
    case 6: moveGroup.setPlannerId("RRTConnectkConfigDefault"); break;
    case 7: moveGroup.setPlannerId("RRTstarkConfigDefault"); break;
    case 8: moveGroup.setPlannerId("TRRTkConfigDefault"); break;
    case 9: moveGroup.setPlannerId("PRMkConfigDefault"); break;
    case 10: moveGroup.setPlannerId("PRMstarkConfigDefault"); break;
    default: moveGroup.setPlannerId("PRMkConfigDefault");
  }
}

// Define planning group
static const std::string PLANNING_GROUP = "ur_manipulator";



class MotionPlanningServer : public rclcpp::Node 
{
public:
    MotionPlanningServer() : Node("motion_planning_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
    {
        using namespace std::placeholders;
        this->joint_space_trajectory_service = this->create_service<motion_planning_interfaces::srv::JointTrajectory>
        ("joint_space_trajectory_service", 
        std::bind(&MotionPlanningServer::jointSpaceTrajectoryExecute, this, _1, _2));

    }

    void jointSpaceTrajectoryExecute(const motion_planning_interfaces::srv::JointTrajectory::Request::SharedPtr request,
                                    const motion_planning_interfaces::srv::JointTrajectory::Response::SharedPtr response){

        
        rclcpp::Logger logger = this->get_logger();
        // rclcpp::NodeOptions nodeOptions;
        // nodeOptions.automatically_declare_parameters_from_overrides(true);
        auto moveGroupNode = rclcpp::Node::make_shared("joint_space_trajectory");

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(moveGroupNode);
        std::thread([&executor]() { executor.spin(); }).detach();

        
        moveit::planning_interface::MoveGroupInterface moveGroup(moveGroupNode, PLANNING_GROUP);
        
        int planner = request->planner;

        setPlanner(moveGroup, planner);
        

        moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

        const moveit::core::JointModelGroup* jointModelGroup = moveGroup.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        moveit::core::RobotStatePtr current_state = moveGroup.getCurrentState(10);

        std::vector<double> jointGroupPositions;
        current_state->copyJointGroupPositions(jointModelGroup, jointGroupPositions);


        // setJointGroupPositionsInDegrees(jointGroupPositions, 50.0, -75.0, 0.0, 0.0, 30.0 ,0.0);
        setJointGroupPositionsInDegrees(jointGroupPositions, request->joints[0], request->joints[1], request->joints[2],
                                        request->joints[3], request->joints[4], request->joints[5]);


        moveGroup.setMaxVelocityScalingFactor(0.70);
        moveGroup.setMaxAccelerationScalingFactor(0.70);

        bool within_bounds = moveGroup.setJointValueTarget(jointGroupPositions);
        if (!within_bounds){
            RCLCPP_WARN(logger, "\n@@@@@@@ Position out of bounds!!! @@@@@@@\n");
        }
        
        auto const [success, plan] = [&moveGroup]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(moveGroup.plan(msg));
            return std::make_pair(ok, msg);
        }();
        
        response->executed = success;

        // Execute the plan

         moveit::core::MoveItErrorCode executeTrajStatus;

        if(success) {
            RCLCPP_INFO(logger, "\n@@@@@@@ Planing succeed!@@@@@@@\n");
            executeTrajStatus = moveGroup.execute(plan);
            executor.cancel();

    
        } else {
            RCLCPP_ERROR(logger, "\n@@@@@@@ Planing failed! @@@@@@@\n");
            executor.cancel();
        }                  
    
    }
    void cartesianSpaceTrajectoryExecute(const motion_planning_interfaces::srv::JointTrajectory::Request::SharedPtr request, const motion_planning_interfaces::srv::JointTrajectory::Response::SharedPtr response,bool isLinearTrajectory = false){
        rclcpp::Logger logger = this->get_logger();
        auto moveGroupNode = rclcpp::Node::make_shared(isLinearTrajectory ? "cartesian_space_trajectory" : "cartesian_space_trajectory_linear");

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(moveGroupNode);
        std::thread([&executor]() { executor.spin(); }).detach();
        
        moveit::planning_interface::MoveGroupInterface moveGroup(moveGroupNode,PLANNING_GROUP);

        int planner = request->planner;
        setPlanner(moveGroup, planner);

        moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
        bool success = false;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if(isLinearTrajectory){
            std::vector<geometry_msgs::msg::Pose> waypoints;
            geometry_msgs::msg::Pose currentPose = moveGroup.getCurrentPose("wrist_3_link").pose;
            waypoints.push_back(currentPose);
            waypoints.push_back(createPoseMsg(request));

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = moveGroup.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
            success = fraction >0 ? true : false;
            plan.trajectory_ = trajectory;


        } else { 
            auto target_pose = createPoseMsg(request);
            moveGroup.setPoseTarget(target_pose,"wrist_3_link");
            success = static_cast<bool>(moveGroup.plan(plan));
        } 
        response->executed = success;
        if(success){
            RCLCPP_INFO(logger, "\n@@@@@@@ Planing succeed!@@@@@@@\n");
            moveGroup.execute(plan);
            executor.cancel();
        }
        else{
            RCLCPP_ERROR(logger, "\n@@@@@@@ Planing failed! @@@@@@@\n");
            executor.cancel();
        }
    }
    void trajectoryPlanningSelector(const motion_planning_interfaces::srv::JointTrajectory::Request::SharedPtr request, const motion_planning_interfaces::srv::JointTrajectory::Response::SharedPtr response){
        rclcpp::Logger logger = this->get_logger();
        RCLCPP_INFO(logger, "\n@@@@@@@ Trajectory Selector @@@@@@@\n");


        // Planner type selectors
        // 0 - joint space planner
        // 1 - cartesian space planner
        // 2 - cartesian space planner - linear trajectory
        int type = request->type;

        if(!type){
            RCLCPP_INFO(logger, "\n@@@@@@@ Joints space planner @@@@@@@\n");
            jointSpaceTrajectoryExecute(request,response);
        }
        if(type == 1){
            RCLCPP_INFO(logger, "\n@@@@@@@ Cartesian space planner @@@@@@@\n");
            cartesianSpaceTrajectoryExecute(request,response,false);
        }
        if(type == 2){
            RCLCPP_INFO(logger, "\n@@@@@@@ Cartesian space planner - lienar trajectory @@@@@@@\n");
            cartesianSpaceTrajectoryExecute(request,response,true);
        }


    }

private:
    rclcpp::Service<motion_planning_interfaces::srv::JointTrajectory>::SharedPtr joint_space_trajectory_service;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanningServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
