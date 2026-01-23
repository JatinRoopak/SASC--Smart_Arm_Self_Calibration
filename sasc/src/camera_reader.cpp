#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;

class CameraReader : public rclcpp::Node {
    public:
        CameraReader() : Node("camera_reader_node"){

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            image_sub = this -> create_subscription<sensor_msgs::msg::Image>(
                "/wrist_camera/image_raw",
                10,
                std::bind(&CameraReader::image_callback, this, std::placeholders::_1)
            );

            info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/wrist_camera/camera_info",
                10,
                std::bind(&CameraReader::info_callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this-> get_logger(), "waiting for camera data....");

            arm_target_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/arm_target_position", 10);
        }

    private:

        cv::Mat camera_matrix;
        cv::Mat dist_coeff;
        bool info_received_ = false;

        std::shared_ptr<tf2_ros::TransformListener>tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer>tf_buffer;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;   
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_target_publisher;

        
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
            if (!info_received_){
                RCLCPP_WARN(this->get_logger(), "Waiting for callibration...");
                return;
            }

            cv_bridge::CvImagePtr cv_ptr;

            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }catch (cv_bridge::Exception& e){
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            auto params = cv::aruco::DetectorParameters::create();

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            cv::aruco::detectMarkers(cv_ptr->image, dict, corners, ids, params);

            if (ids.size() > 0) {
                std::vector<cv::Vec3d> rvecs, tvecs;

                cv::aruco::estimatePoseSingleMarkers(corners, 0.20, camera_matrix, dist_coeff, rvecs, tvecs);

                for (size_t i = 0; i<ids.size(); i++){
                    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
                    cv::drawFrameAxes(cv_ptr->image, camera_matrix, dist_coeff, rvecs[i], tvecs[i], 0.1);

                    geometry_msgs::msg::PoseStamped local_pose;
                    local_pose.header.frame_id = "camera_link_optical";
                    local_pose.header.stamp = msg->header.stamp;

                    local_pose.pose.position.x = tvecs[i][0];
                    local_pose.pose.position.y = tvecs[i][1];
                    local_pose.pose.position.z = tvecs[i][2];

                    local_pose.pose.orientation.w = 1.0;

                    try{
                        if (tf_buffer->canTransform("base_link", "camera_link_optical", msg->header.stamp, rclcpp::Duration::from_seconds(0.1))){
                            geometry_msgs::msg::PoseStamped global_pose;
                            tf_buffer->transform(local_pose, global_pose, "base_link"); //main transformation to find coordinate in base_link frame

                            //RCLCPP_INFO(this->get_logger(), "Global pose found X: %.2f, Y: %.2f, Z: %.2f", global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z);
                            arm_target_publisher->publish(global_pose);
                        }
                    } catch (tf2::TransformException &ex){
                        RCLCPP_WARN(this->get_logger(), "Transform Error: %s", ex.what());
                    }
                    RCLCPP_INFO(this->get_logger(), "Marker %d Pos: [X: %.3f, Y: %.3f, Z: %.3f]", ids[i], tvecs[i][2], tvecs[i][0], tvecs[i][1]);
                }
            }
            cv::imshow("3D Robot Vision", cv_ptr->image);
            cv::waitKey(1);
        }

        void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
            if (info_received_) return;

            camera_matrix = cv::Mat(3, 3, CV_64F);

            for (int i=0; i<9; i++) {
                camera_matrix.at<double>(i/3, i%3) = msg->k[i];
            }

            dist_coeff = cv::Mat(msg->d).clone();

            info_received_ =  true;
            RCLCPP_INFO(this->get_logger(), "Calibration Recieved fx=%.2f cx=%.2f", camera_matrix.at<double>(0,0), camera_matrix.at<double>(0,2));
        }
};


int main(int argc, char*argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraReader>());
    rclcpp::shutdown();
    return 0;
}

