#include "custom_guyi/ros2node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

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

Ros2Node::Ros2Node()
  : rclcpp::Node("ros2_node"), depth(0.0f)
{
    imu = "Not Active";
    orin = "Not Active";
    sonar = "Not Active";


    this->declare_parameter<std::string>("device", "");
    std::string device;
    this->get_parameter("device", device);

    this->declare_parameter<std::string>("cam_topic", "/debayer/image_raw/rgb");
    this->declare_parameter<std::string>("depth_topic", "/bar30/depth");
    this->declare_parameter<std::string>("sonar_topic", "/imagenex831l/sonar_health");
    this->declare_parameter<std::string>("imu_topic", "/imu/data");

    std::string cam_topic;
    std::string depth_topic;
    std::string sonar_topic;
    std::string imu_topic;
    std::string orin_topic;

    this->get_parameter("cam_topic", cam_topic);
    this->get_parameter("depth_topic", depth_topic);
    this->get_parameter("sonar_topic", sonar_topic);
    this->get_parameter("imu_topic", imu_topic);

    if(device == "jetson_1") {
      orin_topic = "jetson_2" + cam_topic;
    } else {
      orin_topic = "jetson_1" + cam_topic;
    }

    cam_topic = device + cam_topic;
    sonar_topic = device + sonar_topic;

    cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        cam_topic, 10, std::bind(&Ros2Node::cam_callback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        depth_topic, 10, std::bind(&Ros2Node::depth_callback, this, std::placeholders::_1));

    sonar_sub_ = this->create_subscription<std_msgs::msg::String>(
        sonar_topic, 10, std::bind(&Ros2Node::sonar_callback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, std::bind(&Ros2Node::imu_callback, this, std::placeholders::_1));

    orin_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        orin_topic, 10, std::bind(&Ros2Node::orin_callback, this, std::placeholders::_1));

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

void Ros2Node::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if(msg != nullptr) {
      imu = "Active";
    } else {
      imu = "Not Active";
    }
}

void Ros2Node::orin_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if(msg != nullptr) {
      orin = "Active";
    } else {
      orin = "Not Active";
    }
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

std::string Ros2Node::getIMU() {
    return imu;
}

std::string Ros2Node::getOrin() {
    return orin;
}

std::string Ros2Node::getBag() {
  std::string directory = ".";
  std::string file = ".db3";
   if(fileExists(directory, file)) {
    return "Active";
   }
   return "Not Active";
}
