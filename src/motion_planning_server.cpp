#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <math.h>


#include "motion_planning_interfaces/srv/joint_trajectory.hpp"

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

        
        static const std::string PLANNING_GROUP = "ur_manipulator";

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
    // void cartesianSpaceTrajectoryExecute(const motion_planning_interfaces::srv::JointTrajectory::Request::SharedPtr request, const motion_planning_interfaces::srv::JointTrajectory::Response::SharedPtr response){
    //     rclcpp::Logger logger = this->get_logger();

    // }
    void trajectoryPlanningSelector(const motion_planning_interfaces::srv::JointTrajectory::Request::SharedPtr request, const motion_planning_interfaces::srv::JointTrajectory::Response::SharedPtr response){
        rclcpp::Logger logger = this->get_logger();
        RCLCPP_INFO(logger, "\n@@@@@@@ Trajectory Selector @@@@@@@\n");

        auto moveGroupNode = rclcpp::Node::make_shared("joint_space_trajectory");

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(moveGroupNode);
        std::thread([&executor]() { executor.spin(); }).detach();
        std::string type = request->type;

        if(type == "pose"){
            RCLCPP_INFO(logger, "\n@@@@@@@ Cartesian space planner @@@@@@@\n");
            response->executed = true;
            return;
        }
        if(type == "joints"){
            RCLCPP_INFO(logger, "\n@@@@@@@ Joints space planner @@@@@@@\n");
            jointSpaceTrajectoryExecute(request,response);
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
