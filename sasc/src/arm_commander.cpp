#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <atomic> //for the boolean operator to stop robot from listening to every target coming 
#include <memory>
#include <thread>
#include <chrono> 

using namespace std;

class ArmCommander : public rclcpp::Node {
    public:
        ArmCommander(const rclcpp::NodeOptions & options) 
        : Node("arm_commander_node", options), ready_to_operate(false), is_moving(false){

            //must recieve an argument to run the custom safety_dist
            if (this->has_parameter("safety_dist")){
                this->get_parameter("safety_dist", safety_dist);
            } else{
                safety_dist = this->declare_parameter<double>("safety_dist", 0.40); //safety distance parameter deafult is 0.40 
            }

            callback_group_subscriber = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive
            );
            auto sub_opt = rclcpp::SubscriptionOptions();
            sub_opt.callback_group = callback_group_subscriber;

            calibration_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sasc/data", 10);

            arm_target_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/arm_target_position",
                rclcpp::SensorDataQoS(),
                std::bind(&ArmCommander::arm_target, this, std::placeholders::_1),
                sub_opt //different lane
            );
        }

        void init_moveit(const std::string & planning_group) {
            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group);

            RCLCPP_INFO(this->get_logger(), "Waiting for connection......");
            bool sucess = move_group->startStateMonitor(5.0);

            if(sucess){
                RCLCPP_INFO(this->get_logger(), "Success: Robot is connected.");
                ready_to_operate = true; 
            }else{
                RCLCPP_ERROR(this->get_logger(), "TimeOut: Robot not responding.");
            }

        }

    private:
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_target_subscriber;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr calibration_publisher;

        double safety_dist;
        std::atomic<bool> ready_to_operate; //flag to let the joint states
        std::atomic<bool> is_moving; //the busy flag

        void arm_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if(!ready_to_operate){
                RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not Inistialized.....");
                return;
            }
            if(!move_group){
                RCLCPP_ERROR(this->get_logger(), "CRITICAL: MoveGroup is NULL!");
                return;
            }
            if (is_moving){
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            RCLCPP_INFO(this->get_logger(), "Target is locked, Executing the move........");

            geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();

            double desired_x = msg->pose.position.x-safety_dist;
            double actual_x = current_pose.pose.position.x;
            double dist_error = abs(desired_x - actual_x);

            if (dist_error < 0.1) {
                auto data_msg = std_msgs::msg::Float64MultiArray();
                // Robot Pose currently
                data_msg.data.push_back(current_pose.pose.position.x);
                data_msg.data.push_back(current_pose.pose.position.y);
                data_msg.data.push_back(current_pose.pose.position.z);
                // Camera Target currently 
                data_msg.data.push_back(msg->pose.position.x);
                data_msg.data.push_back(msg->pose.position.y);
                data_msg.data.push_back(msg->pose.position.z);
                
                calibration_publisher->publish(data_msg);
                return;
            }

            is_moving = true;

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
    node_options.automatically_declare_parameters_from_overrides(true); 
    auto node = std::make_shared<ArmCommander>(node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    node->init_moveit("irb6640_arm");

    while(rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}