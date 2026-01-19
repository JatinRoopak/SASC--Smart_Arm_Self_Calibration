#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <atomic> //for the boolean operator to stop robot from listening to every target coming 
#include <memory>
#include <thread>
#include <chrono> 

using namespace std;

class ArmCommander : public rclcpp::Node {
    public:
        ArmCommander(const rclcpp::NodeOptions & options) 
        : Node("arm_commander_node", options), robotLoadUp(false), isRobotMoving(false){

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
                sub_opt //different lane for subscriber
            );

            correction_subscriber = this->create_subscription<std_msgs::msg::Float64>(
                "/sasc/correction",
                10,
                std::bind(&ArmCommander::correction_callback, this, std::placeholders::_1),
                sub_opt
            );
        }

        void init_moveit(const std::string & planning_group) {
            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group);

            RCLCPP_INFO(this->get_logger(), "Waiting for connection......");
            bool sucess = move_group->startStateMonitor(5.0);

            if(sucess){
                RCLCPP_INFO(this->get_logger(), "Success: Robot is connected.");
                robotLoadUp = true; 
            }else{
                RCLCPP_ERROR(this->get_logger(), "TimeOut: Robot not responding.");
            }

        }

    private:
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_target_subscriber;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr calibration_publisher;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr correction_subscriber;

        double safety_dist;
        std::atomic<bool> robotLoadUp; //flag to let the robot joint states fully loaded
        std::atomic<bool> isRobotMoving; //the busy flag
        int sample_sent = 0; //number of samples of position sended (after 10 mean is taken and error is found out)
        bool force_move = false;

        bool calibration_complete = false; //final bool to flag completition of process for launch file use

        void arm_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if(!robotLoadUp){
                RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not Inistialized.....");
                return;
            }
            if(!move_group){
                RCLCPP_ERROR(this->get_logger(), "CRITICAL: MoveGroup is NULL!");
                return;
            }
            if (isRobotMoving){
                return;
            }
            if (calibration_complete){
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            RCLCPP_INFO(this->get_logger(), "Target is locked, Executing the move........");

            geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();

            double desired_x = msg->pose.position.x-safety_dist;
            double actual_x = current_pose.pose.position.x;
            double dist_error = abs(desired_x - actual_x);

            if (dist_error < 0.1 && !force_move) {

                if(sample_sent<50){
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
                    sample_sent++;
                    RCLCPP_INFO(this->get_logger(), "Collecting Data [%d/50]...", sample_sent);
                }
                else{
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,"Batch Complete. Waiting for Clibration node to send correction....");
                }
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Target is locked with error = %.3f. Moving....", dist_error);
            
            isRobotMoving = true;
            force_move = false;

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

            isRobotMoving = false;
        }

        void correction_callback(const std_msgs::msg::Float64::SharedPtr msg){
            double correction_val = msg->data;
            RCLCPP_INFO(this->get_logger(), "Correction recieved: %.5f m", correction_val);
            if (abs(correction_val) < 0.001){
                RCLCPP_INFO(this->get_logger(), "Calibration completed...........");

                safety_dist = safety_dist + correction_val;
                calibration_complete = true;
                return;
            }

            safety_dist = safety_dist + correction_val;
            sample_sent = 0;
            force_move = true;
            RCLCPP_INFO(this->get_logger(), "Correction applied.Force Relaignment....");
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