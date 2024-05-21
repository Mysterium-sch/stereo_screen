#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

class Ros2Node : public rclcpp::Node
{
	public:
		Ros2Node();
		cv_bridge::CvImagePtr cv_ptr;
		cv_bridge::CvImagePtr getRosMsg();

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
		void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};