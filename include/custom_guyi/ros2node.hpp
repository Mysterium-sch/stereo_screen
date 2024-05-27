#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/Float32.hpp"
#include <cv_bridge/cv_bridge.h>
//from imagenex831l_ros2.msg import ProcessedRange

class Ros2Node : public rclcpp::Node
{
	public:
		Ros2Node();
		cv::Mat cv_ptr;
		cv::Mat getRosMsg();
		float deph;
		float getDepth()

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
		//rclcpp::Subscription<ProcessedRange>::SharedPtr sonar_sub_;
		//rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imu_sub_;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
		//rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr orin_sub;
		//rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bag_sub;

		void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg);
		void depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
};