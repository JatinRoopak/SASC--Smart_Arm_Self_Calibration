#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <atomic> //for the boolean operator to stop robot from listening to every target coming 
#include <memory>

using namespace std;

class ArmCommander : public rclcpp::Node {
    public:
        ArmCommander(const rclcpp::NodeOptions & options) 
        : Node("arm_commander_node", options), is_moving(false){

            //must recieve an argument to run the custom safety_dist
            if (this->has_parameter("safety_dist")){
                this->get_parameter("safety_dist", safety_dist);
            } else{
                safety_dist = this->declare_parameter<double>("safety_dist", 0.40); //safety distance parameter deafult is 0.40 
            }

            rclcpp::QoS qos_profile(1); //focus on the last sample
            qos_profile.best_effort();

            arm_target_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/arm_target_position",
                rclcpp::SensorDataQoS(),
                std::bind(&ArmCommander::arm_target, this, std::placeholders::_1)
            );
        }

        void init_moveit(const std::string & planning_group) {
            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group);
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_target_subscriber;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        double safety_dist;

        std::atomic<bool> is_moving; //the busy flag

        void arm_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if(!move_group){
                RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not Inistialized.....");
                return;
            }

            if (is_moving){
                return;
            }

            is_moving = true;
            RCLCPP_INFO(this->get_logger(), "Target is locked, Executing the move........");

            geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();
            geometry_msgs::msg::PoseStamped safe_target = *msg;
            safe_target.pose.position.x -= safety_dist; //adding safety distance
            safe_target.pose.orientation = current_pose.pose.orientation; //Using robot current orientation 

            move_group->setPoseTarget(safe_target);
            auto result = move_group->move();

            if(result == moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_INFO(this->get_logger(), "Movement done");
            }else{
                RCLCPP_INFO(this->get_logger(), "Movement failed!");
            }

            is_moving = false;
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true); //Auto declare the parameters
    auto node = std::make_shared<ArmCommander>(node_options);
    node->init_moveit("irb6640_arm");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}