#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp" 
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

sensor_msgs::msg::Image Mat_to_ImageMsg(const cv::Mat& image, rclcpp::Clock::SharedPtr clock)
{
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = clock->now();
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
       rclcpp::Clock::SharedPtr clock_;

       void timer_callback()
       {
         // 定时读取摄像头帧并发布
         cv::Mat frame;
         cap >> frame;
           if(frame.empty())
           {
               RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty frame captured.");
               return;
           }
           auto msg = Mat_to_ImageMsg(frame, clock_);
           image_pub_->publish( msg);
       }
public:
   Camera_Node(std::string name) : Node(name)
   {
       // 打开默认摄像头（索引 0）
       cap.open(0);
       if(!cap.isOpened())
       {
           RCLCPP_ERROR(this->get_logger(), "Failed to open camera. Shutting down.");
           rclcpp::shutdown();
           return;
       }
       RCLCPP_INFO(this->get_logger(), "%s has been started.", name.c_str());

       clock_ = this->get_clock();

       // 发布原始图像话题
       image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("OriginalImage", 10);

       // 约 30Hz 发布频率（33ms 一帧）
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
