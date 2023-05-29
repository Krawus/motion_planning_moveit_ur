#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/utils/moveit_error_code.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


class MotionPlanningService : public rclcpp::Node {
    public:
    MotionPlanningService() : Node("ur_planning_service",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){
        
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryControlServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}