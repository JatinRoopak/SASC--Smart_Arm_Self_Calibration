#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>

using namespace std;

class ErrorCalibration : public rclcpp::Node{
    public:
        ErrorCalibration() : Node("error_calibration_node"){
            this->declare_parameter<double>("safety_distnace_x", 0.40); //ROS parameter for safety dist ()/default is 0.40m
            this->get_parameter("safety_distnace_x", safety_distnace_x);

            this->declare_parameter<double>("safety_distnace_y", 0.00); //ROS parameter for safety dist ()/default is 0.40m
            this->get_parameter("safety_distnace_y", safety_distnace_y);

            this->declare_parameter<double>("safety_distnace_z", 0.00); //ROS parameter for safety dist ()/default is 0.40m
            this->get_parameter("safety_distnace_z", safety_distnace_z);
            // RCLCPP_INFO(this->get_logger(), "Saftey dist is set to: %.2fm", safety_distnace_x);
            
            sasc_data_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/sasc/data",
                10,
                std::bind(&ErrorCalibration::info_callback, this, placeholders::_1)
            );

            sasc_correction_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sasc/correction", 10);
        }
    private:
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sasc_data_subscriber;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sasc_correction_publisher;
        vector<vector<double>> data_history;
        double safety_distnace_x;
        double safety_distnace_y;
        double safety_distnace_z;

        void info_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            data_history.push_back(msg->data);
            int count = data_history.size();
            RCLCPP_INFO(this->get_logger(), "Collected sameple [%d/10]", count);

            if (count>=10){
                calculate_accuracy();
                data_history.clear();
                RCLCPP_INFO(this->get_logger(), "Batch completed, waiting for verification....");
            }
        }

        void calculate_accuracy(){
            RCLCPP_INFO(this->get_logger(), "Batch of data if full. Calculating Error");
            double total_error_x = 0.0;
            double total_error_y = 0.0;
            double total_error_z = 0.0;

            for (const auto& row : data_history){
                //// Data format: [rob_x, rob_y, rob_z, cam_x, cam_y, cam_z]
                double robot_x = row[0]; 
                double robot_y = row[1];
                double robot_z = row[2]; 

                double cam_x = row[3];
                double cam_y = row[4];
                double cam_z = row[5];

                double target_x = cam_x - safety_distnace_x;
                double error_x = robot_x - target_x;
                total_error_x += error_x;

                double target_y = cam_y - safety_distnace_y;
                double error_y = robot_y - target_y;
                total_error_y += error_y;

                double target_z = cam_z - safety_distnace_z;
                double error_z = robot_z - target_z;
                total_error_z += error_z;
            }

            double mean_error_x = total_error_x / data_history.size(); //publishing message
            double mean_error_y = total_error_y / data_history.size(); //publishing message
            double mean_error_z = total_error_z / data_history.size(); //publishing message


            auto error_message = std_msgs::msg::Float64MultiArray();
            error_message.data.push_back(mean_error_x);
            error_message.data.push_back(mean_error_y);
            error_message.data.push_back(mean_error_z);

            sasc_correction_publisher->publish(error_message);
            RCLCPP_INFO(this->get_logger(), ">> CORRECTION SENT: X=%.5f, Y=%.5f, Z=%.5f", mean_error_x, mean_error_y, mean_error_z);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); //time for message to get published
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ErrorCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}