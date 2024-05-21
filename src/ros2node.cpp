#include "custom_guyi/ros2node.hpp"

Ros2Node::Ros2Node()
  : rclcpp::Node("ros2_node")
{
  cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10, std::bind(&Ros2Node::cam_callback, this, std::placeholders::_1));
}

  void Ros2Node::cam_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }

  cv_bridge::CvImagePtr Ros2Node::getRosMsg() {
    return cv_ptr;
  }
