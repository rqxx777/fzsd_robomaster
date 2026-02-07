#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp" 
#include <cv_bridge/cv_bridge.hpp>
sensor_msgs::msg::Image Mat_to_ImageMsg(const cv::Mat& image)
{
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = rclcpp::Clock().now();
    cv_image.header.frame_id = "camera_frame";
    cv_image.encoding = "bgr8";
    cv_image.image = image;
    return *cv_image.toImageMsg();
}
class Camera_Node : public rclcpp::Node
{
 private:
        rclcpp::TimerBase::SharedPtr timer_;
        cv::VideoCapture cap;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        void timer_callback()
        {
          cv::Mat frame;
          cap >> frame;
            if(frame.empty())   
            {
                RCLCPP_WARN(this->get_logger(), "Empty frame captured.");
                return;
            }
            auto msg = Mat_to_ImageMsg(frame);
            image_pub_->publish( msg);
            //RCLCPP_INFO(this->get_logger(), "Published an image frame.");
        }
 public:
    Camera_Node(std::string name) : Node(name)
    {
        cap.open(0);
        if(!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "%s has been started.", name.c_str());
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("OriginalImage", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&Camera_Node::timer_callback, this));
    }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Camera_Node>("camera_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
}