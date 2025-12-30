#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std;

class CameraReader : public rclcpp::Node {
    public:
        CameraReader() : Node("camera_reader_node"){
            subscription_ = this -> create_subscription<sensor_msgs::msg::Image>(
                "/wrist_camera/image_raw",
                10,
                std::bind(&CameraReader::topic_callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this-> get_logger(), "waiting for camera data....");
        }

    private:
        void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            }catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            int height = cv_ptr->image.rows;
            int width = cv_ptr->image.cols;

            static int count = 0;
            if (count%30 == 0){
                RCLCPP_INFO(this->get_logger(), "Received Image: %dx%d pixels", width, height);

            }
            count++;
        }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        
};


int main(int argc, char*argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraReader>());

    rclcpp::shutdown();
    return 0;
}

