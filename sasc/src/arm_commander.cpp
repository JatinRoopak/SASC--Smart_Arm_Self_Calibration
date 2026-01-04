#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

class ArmCommander : public rclcpp::Node {
    public:
        ArmCommander(const rclcpp::NodeOptions & options) : Node("arm_commander_node", options){
            arm_target_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/arm_target_position",
                10,
                std::bind(&ArmCommander::arm_target, this, std::placeholders::_1)
            );
        }

        void init_moveit(const std::string & planning_group) {
            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group);
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_target_subscriber;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

        void arm_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const{
            if(!move_group){
                RCLCPP_ERROR(this->get_logger(), "MOveGroupInterface not Inistialized.....");
                return;
            }

            move_group->setPoseTarget(*msg);

            auto result = move_group->move();

            if(result == moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_INFO(this->get_logger(), "Movement done");
            }else{
                RCLCPP_INFO(this->get_logger(), "Movement failed!");
            }
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ArmCommander>(node_options);
    node->init_moveit("irb6640_arm");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}