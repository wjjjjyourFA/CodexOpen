#ifndef AMPHIBIOUS_CHASSIS_STATUS_CONVERT_H
#define AMPHIBIOUS_CHASSIS_STATUS_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/amphibious_chassis_status.hpp"
#include "dds/msg/AmphibiousChassisStatus.hpp"

class ROS2_To_DDS_AmphibiousChassisStatus : public SubscribeROS2Msg<ros2::msg::AmphibiousChassisStatus>,
	public PublishDDSMsg<dds::msg::AmphibiousChassisStatus>
{
public:
	ROS2_To_DDS_AmphibiousChassisStatus(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::AmphibiousChassisStatus>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::AmphibiousChassisStatus>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::AmphibiousChassisStatus &ros2_msg)
		{
			dds::msg::AmphibiousChassisStatus dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.us_speed() = ros2_msg.us_speed;
			dds_msg.us_curve() = ros2_msg.us_curve;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_AmphibiousChassisStatus : public SubscribeDDSMsg<dds::msg::AmphibiousChassisStatus>, 
	public PublishROS2Msg<ros2::msg::AmphibiousChassisStatus>
{
public:
	DDS_To_ROS2_AmphibiousChassisStatus(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::AmphibiousChassisStatus>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::AmphibiousChassisStatus>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::AmphibiousChassisStatus &dds_msg)
		{
			ros2::msg::AmphibiousChassisStatus ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.us_speed = dds_msg.us_speed();
			ros2_msg.us_curve = dds_msg.us_curve();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // AMPHIBIOUS_CHASSIS_STATUS_CONVERT_H