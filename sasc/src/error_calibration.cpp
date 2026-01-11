#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>

using namespace std;

class ErrorCalibration : public rclcpp::Node{
    public:
        ErrorCalibration() : Node("error_calibration_node"){
            this->declare_parameter<double>("safety_dist", 0.40); //ROS parameter for safety dist ()/default is 0.40m
            this->get_parameter("safety_dist", safety_dist);
            // RCLCPP_INFO(this->get_logger(), "Saftey dist is set to: %.2fm", safety_dist);
            
            sasc_data_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/sasc/data",
                10,
                std::bind(&ErrorCalibration::info_callback, this, placeholders::_1)
            );
        }
    private:
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sasc_data_subscriber;
        vector<vector<double>> data_history;
        double safety_dist;

        void info_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            data_history.push_back(msg->data);
            int count = data_history.size();
            RCLCPP_INFO(this->get_logger(), "Collected sameple [%d/10]", count);

            if (count>=10){
                calculate_accuracy();
                rclcpp::shutdown();
            }
        }

        void calculate_accuracy(){
            RCLCPP_INFO(this->get_logger(), "Batch of data if full. Calculating Error");
            double total_squared_error = 0.0;

            for (const auto& row : data_history){
                //first 3 elements in data are robot's coordinates and next 3 are camera's
                double robot_x = row[0]; 
                double cam_x = row[3];

                double target_x = cam_x - safety_dist;
                double error = robot_x - target_x;
                total_squared_error += std::pow(error, 2);
            }

            double mean_squared_error = total_squared_error / data_history.size();
            double rms = std::sqrt(mean_squared_error);

            RCLCPP_INFO(this->get_logger(), "---------------------------------------");
            RCLCPP_INFO(this->get_logger(), "   FINAL ACCURACY REPORT");
            RCLCPP_INFO(this->get_logger(), "   Safety Distance Used: %.2fm", safety_dist);
            RCLCPP_INFO(this->get_logger(), "   RMSE (Average Error): %.5f meters", rms);
            RCLCPP_INFO(this->get_logger(), "   Error in mm: %.2f mm", rms * 1000.0);
            RCLCPP_INFO(this->get_logger(), "---------------------------------------");
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ErrorCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}