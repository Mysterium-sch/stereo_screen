#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/string.hpp"

class Ros2Node : public rclcpp::Node
{
	public:
		Ros2Node();
		cv::Mat cv_ptr;
		cv::Mat getRosMsg();
		float deph;
		float getDepth()
		std::string sonar;
		std::string getSonar();

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sonar_sub_;
		//rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imu_sub_;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
		//rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr orin_sub;
		//rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bag_sub;

		void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg);
		void depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
		void sonar_callback(const std_msgs::msg::String::sharedPtr msg);
};