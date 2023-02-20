#include "add_depth_to_detection_node.hpp"
#include <nlohmann/json.hpp>
#include <math.h>
#include <unistd.h>

/**
 * @brief Contructor.
 */
AddDepthToDetectionNode::AddDepthToDetectionNode() : Node("add_depth_to_detection_node", rclcpp::NodeOptions().use_intra_process_comms(false))
{
	this->declare_parameter("in_depthframe_topic", "");
	this->declare_parameter("in_detection_topic", "");
	this->declare_parameter("qos_sensor_data", true);
	this->declare_parameter("qos_history_depth", 10);

}

/**
 * @brief Initialize image node.
 */
void AddDepthToDetectionNode::init()
{
	bool qos_sensor_data;
	std::string in_depthframe_topic, in_detection_topic;
	int qos_history_depth;

	this->get_parameter("in_depthframe_topic", in_depthframe_topic);
	this->get_parameter("in_detection_topic", in_detection_topic);
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);

	if(qos_sensor_data){
		std::cout << "using ROS2 qos_sensor_data" << std::endl;
		m_qos_profile = rclcpp::SensorDataQoS();
	}

	m_qos_profile = m_qos_profile.keep_last(qos_history_depth);
	//m_qos_profile = m_qos_profile.lifespan(std::chrono::milliseconds(500));
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	

	m_qos_profile_sysdef = m_qos_profile_sysdef.keep_last(qos_history_depth);
	//m_qos_profile_sysdef = m_qos_profile_sysdef.lifespan(std::chrono::milliseconds(500));
	m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

	std::string logstr = "creating image subscription for " + in_depthframe_topic ;
	RCLCPP_INFO(this->get_logger(), logstr.c_str());
	m_depth_subscription = this->create_subscription<sensor_msgs::msg::Image>( in_depthframe_topic, m_qos_profile, std::bind(&AddDepthToDetectionNode::frameCallback, this, std::placeholders::_1));
	//cv::namedWindow(m_window_name_depth, cv::WINDOW_AUTOSIZE);

	sleep(1);

	logstr = "creating detection subscription for " + in_detection_topic ;
	RCLCPP_INFO(this->get_logger(), logstr.c_str());
	m_detection_subscription 	= this->create_subscription<std_msgs::msg::String>( in_detection_topic, m_qos_profile, std::bind(&AddDepthToDetectionNode::objectCallback, this, std::placeholders::_1));
	
	logstr = "publishing to " + in_detection_topic + "_with_distance";
	RCLCPP_INFO(this->get_logger(), logstr.c_str());
	m_detection_publisher   	= this->create_publisher<std_msgs::msg::String>(in_detection_topic + "_with_distance", m_qos_profile_sysdef);

}


void AddDepthToDetectionNode::objectCallback(std_msgs::msg::String::SharedPtr obj_msg)
{
	if (m_image_present)
	{
		nlohmann::json obj_json = nlohmann::json::parse(obj_msg->data.data());		
		try{
			for (auto& [key, val] : obj_json.items()){
				if (!val.is_array())
					continue;

				int val_amount = val.size();

				for(int i = 0; i <  val_amount; i++){

					if(obj_json[key][i].contains("center"))
					{
						double center_x = obj_json[key][i]["center"][0];
						double center_y = obj_json[key][i]["center"][1];
						cv::Size sz = m_last_depth_image.size();
						int x = int(center_x * sz.width);
						int y = int(center_y * sz.height);
	
						int dist_mm = (int(m_last_depth_image.at<uint16_t>(y,x)));// * 0.001); 
						obj_json[key][i]["distance"] = dist_mm;
					}
				}
			}

		} catch (...) {
			RCLCPP_INFO(this->get_logger(), "get the object distance has failed somehow...");
		}

		auto message = std_msgs::msg::String();
		message.data = obj_json.dump().c_str();
		try{
			m_detection_publisher->publish(message);
		}
		catch (...) {
			RCLCPP_INFO(this->get_logger(), "hmm publishing dets has failed!! ");
		}
	}
}


/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void AddDepthToDetectionNode::frameCallback(sensor_msgs::msg::Image::SharedPtr img_msg)
{
	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_16U, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP); 
	//cv::setWindowTitle(m_window_name_depth, std::to_string(m_loop_duration_depth));
	//cv::setWindowTitle(m_window_name, std::to_string(0.0));

	m_last_depth_image = color_image;

	usleep(10);

	m_image_present = true;

	//cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	//imshow(m_window_name_depth, color_image);

	//if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_depth, cv::WND_PROP_AUTOSIZE) >= 0))
	//	rclcpp::shutdown();

	//if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_depth, cv::WND_PROP_AUTOSIZE) >= 0))
	//	rclcpp::shutdown();

}

