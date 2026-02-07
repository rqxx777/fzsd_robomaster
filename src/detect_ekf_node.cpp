#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp" 
#include <cv_bridge/cv_bridge.hpp>
#include "visualization_msgs/msg/marker.hpp"       
#include "visualization_msgs/msg/marker_array.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "armor_detect.hpp"
#include "EKF.hpp"
using std::placeholders::_1;

cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        return cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return cv::Mat(); 
    }
}

class AromorDetectEKFNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pub_;
        EKF ekf{0.033}; 
        ArmorDetector detector;
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            cv::Mat image = imageMsgToMat(msg);
            if(image.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty image.");
                return;
            }
            //RCLCPP_INFO(this->get_logger(), "Received an image");
            std::vector<Detection> detections;
            detector.detect(image, detections);
            visualization_msgs::msg::MarkerArray markers;
            int id = 0;
            for (const auto &d : detections)
            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "world";
                m.header.stamp = this->get_clock()->now();
                m.ns = "armor";
                m.id = id++;
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.action = visualization_msgs::msg::Marker::ADD;
                double u = d.box.x + d.box.width/2.0;
                double v = d.box.y + d.box.height/2.0;
// ======================== 相机 =======================================================
                double fx = 1000;
                double cx_ima=640;
                double cy_ima=360;
// ======================== 装甲板 =====================================================                
                double W = 0.135; 
                double w_pixel =d.box.width;
                double Z = W * fx / w_pixel;
                
                double X = (u - cx_ima) * Z / fx;
                double Y = (v - cy_ima) * Z / fx;
                m.pose.position.x = X;
                m.pose.position.y = Y;
                m.pose.position.z = Z;
                m.pose.orientation.x = 0.0;
                m.pose.orientation.y = 0.0;
                m.pose.orientation.z = 0.0;
                m.pose.orientation.w = 1.0;
                m.scale.x = 0.135;
                m.scale.y = 0.055;
                m.scale.z = 0.02;
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
                m.lifetime = rclcpp::Duration::from_seconds(0.1);
                markers.markers.push_back(m);
            }
            marker_pub_->publish(markers);
            if(!detections.empty())
            {
                auto& d = detections[0]; 
                double u = d.box.x + d.box.width / 2.0;
                double v = d.box.y + d.box.height / 2.0;
                double fx = 1000;       
                double cx_img = 640;    
                double cy_img = 360;
                double W = 0.135;       
                double w_pixel = d.box.width;
                double Z = W * fx / w_pixel;
                double X = (u - cx_img) * Z / fx;
                double Y = (v - cy_img) * Z / fx;
                ekf.updateXYZ(X, Y, Z);
            }
            ekf.predict();
            auto pred = ekf.getPosition();
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.frame_id = "world";
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.pose.position.x = pred.x;
            pose_msg.pose.position.y = pred.y;
            pose_msg.pose.position.z = pred.z;
            pose_msg.pose.orientation.w = 1.0;
            ekf_pub_->publish(pose_msg);
        }
    public:
        AromorDetectEKFNode(std::string name) : Node(name),detector("/src/armor_detect_ekf/src/best.onnx", true)
        {
            RCLCPP_INFO(this->get_logger(), "%s has been started.", name.c_str());
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "OriginalImage", 10, std::bind(&AromorDetectEKFNode::imageCallback, this,_1));
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_markers", 10);
            ekf_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf_position", 10);
        }   
};
int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<AromorDetectEKFNode>("detect_ekf_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
}