#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmCommander : public rclcpp::Node {
    public:
        ArmCommander() : Node("arm_commander_node"){
            RCLCPP_INFO(this->get_logger(), "Arm Commander is running");
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmCommander>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}