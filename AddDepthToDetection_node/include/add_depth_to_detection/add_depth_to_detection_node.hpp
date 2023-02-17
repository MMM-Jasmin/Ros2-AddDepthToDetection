#pragma once
// SYSTEM
#include <chrono>
#include <iostream>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Image viewer node class for receiving and visualizing fused image.
 */
class AddDepthToDetectionNode : public rclcpp::Node
{
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

public:
	AddDepthToDetectionNode();
	void init();

private:

	void frameCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	void objectCallback(std_msgs::msg::String::SharedPtr img_msg);

	std::string m_window_name_depth	 = "Depth_Frame";
	std::string m_detection_key = "";

	cv::Mat m_last_depth_image;
	int m_width, m_height;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_detection_subscription;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_detection_publisher 	= nullptr;



	rclcpp::QoS m_qos_profile = rclcpp::SystemDefaultsQoS();
	rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();

};
