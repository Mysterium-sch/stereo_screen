#include "custom_guyi/ros2node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>

bool fileExists(const std::string& directory, const std::string& extension) {
    try {
        boost::filesystem::recursive_directory_iterator it(directory), end;
        while (it != end) {
            if (boost::filesystem::is_regular_file(*it) && it->path().extension() == extension) {
                return true;
            }
            ++it;
        }
    } catch (const boost::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
    return false;
}

std::string jetsonCheck(const std::string& device) {
    int x = -1;
    if (device == "jetson_1") {
        x = system("ping -c1 -s1 192.168.0.100 > /dev/null 2>&1");
    } else if (device == "jetson_2") {
        x = system("ping -c1 -s1 192.168.0.150 > /dev/null 2>&1");
    }
    return (x == 0) ? "Active" : "Not Active";
}

Ros2Node::Ros2Node()
  : rclcpp::Node("ros2_node"), depth(0.0f), cv_ptr(nullptr)
{
    imu = "Not Active";
    orin = "Not Active";
    sonar = "Not Active";

    this->declare_parameter<std::string>("device", "");
    this->get_parameter("device", device);

    this->declare_parameter<std::string>("cam_topic", "debayer/image_raw/compressed");
    this->declare_parameter<std::string>("depth_topic", "bar30/depth");
    this->declare_parameter<std::string>("sonar_topic", "imagenex831l/sonar_health");
    this->declare_parameter<std::string>("imu_topic", "imu/data");

    std::string cam_topic;
    std::string depth_topic;
    std::string sonar_topic;
    std::string imu_topic;

    this->get_parameter("cam_topic", cam_topic);
    this->get_parameter("depth_topic", depth_topic);
    this->get_parameter("sonar_topic", sonar_topic);
    this->get_parameter("imu_topic", imu_topic);

    cam_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        cam_topic, 10, std::bind(&Ros2Node::cam_callback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        depth_topic, 10, std::bind(&Ros2Node::depth_callback, this, std::placeholders::_1));

    sonar_sub_ = this->create_subscription<std_msgs::msg::String>(
        sonar_topic, 10, std::bind(&Ros2Node::sonar_callback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, std::bind(&Ros2Node::imu_callback, this, std::placeholders::_1));
}

void Ros2Node::cam_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
}

void Ros2Node::depth_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    depth = msg->data;
}

void Ros2Node::sonar_callback(const std_msgs::msg::String::SharedPtr msg) {
    sonar = msg->data;
}

void Ros2Node::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu = (msg != nullptr) ? "Active" : "Not Active";
}

cv::Mat Ros2Node::getRosMsg() {
    return cv_ptr ? cv_ptr->image : cv::Mat();
}

std::string Ros2Node::getDepth() {
    return (depth_sub_->get_publisher_count() > 0) ? std::to_string(depth) : "Not Active";
}

std::string Ros2Node::getSonar() {
    return (sonar_sub_->get_publisher_count() > 0) ? sonar : "Not Active";
}

std::string Ros2Node::getIMU() {
    return (imu_sub_->get_publisher_count() > 0) ? imu : "Not Active";
}

std::string Ros2Node::getOrin() {
    orin = jetsonCheck(device);
    return orin;
}

std::string Ros2Node::getBag() {
    std::string directory = ".";
    std::string file = ".db3";
    return fileExists(directory, file) ? "Active" : "Not Active";
}
