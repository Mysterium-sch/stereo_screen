#include "custom_guyi/ros2node.hpp"

Ros2Node::Ros2Node()
  : rclcpp::Node("ros2_node")
{
      this->declare_parameter<std::string>("cam_topic", "/flir_camera/image_raw");
    std::string cam_topic;
    this->get_parameter("cam_topic", cam_topic);

  cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    cam_topic, 10, std::bind(&Ros2Node::cam_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("pub_topic",10);
}

  void Ros2Node::cam_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
    publisher_->publish(*msg);
  }

  cv::Mat Ros2Node::getRosMsg() {
    return cv_ptr;
  }
