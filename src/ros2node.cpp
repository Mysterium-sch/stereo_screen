#include "custom_guyi/ros2node.hpp"
#include <sensor_msgs/image_encodings.hpp>

Ros2Node::Ros2Node()
  : rclcpp::Node("ros2_node"), depth(0.0f)
{
    this->declare_parameter<std::string>("device", "");
    std::string device;
    this->get_parameter("device", device);

    this->declare_parameter<std::string>("cam_topic", "/flir_camera/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/bar30/depth");
    this->declare_parameter<std::string>("sonar_topic", "/imagenex831l/sonar_health");

    std::string cam_topic;
    std::string depth_topic;
    std::string sonar_topic;

    this->get_parameter("cam_topic", cam_topic);
    this->get_parameter("depth_topic", depth_topic);
    this->get_parameter("sonar_topic", sonar_topic);

    cam_topic = device + cam_topic;
    sonar_topic = device + sonar_topic;

    cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        cam_topic, 10, std::bind(&Ros2Node::cam_callback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        depth_topic, 10, std::bind(&Ros2Node::depth_callback, this, std::placeholders::_1));

    sonar_sub_ = this->create_subscription<std_msgs::msg::String>(
        sonar_topic, 10, std::bind(&Ros2Node::sonar_callback, this, std::placeholders::_1));
}

void Ros2Node::cam_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
}

void Ros2Node::depth_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    depth = msg->data;
}

void Ros2Node::sonar_callback(const std_msgs::msg::String::SharedPtr msg) {
    sonar = msg->data;
}

cv::Mat Ros2Node::getRosMsg() {
    return cv_ptr;
}

float Ros2Node::getDepth() {
    return depth;
}

std::string Ros2Node::getSonar() {
    return sonar;
}
