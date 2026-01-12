#ifndef SOFTWARE_VERSION_CONVERT_H
#define SOFTWARE_VERSION_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/software_version.hpp"
#include "dds/msg/SoftwareVersion.hpp"

class ROS2_To_DDS_SoftwareVersion : public SubscribeROS2Msg<ros2::msg::SoftwareVersion>,
	public PublishDDSMsg<dds::msg::SoftwareVersion>
{
public:
	ROS2_To_DDS_SoftwareVersion(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::SoftwareVersion>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::SoftwareVersion>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::SoftwareVersion &ros2_msg)
		{
			dds::msg::SoftwareVersion dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.us_msg_data_length() = ros2_msg.us_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;

			{int size = ros2_msg.uc_ver.size();
			for(int i = 0; i < size; ++i)
			{
				dds_msg.uc_ver()[i] = ros2_msg.uc_ver[i];
			}}
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_SoftwareVersion : public SubscribeDDSMsg<dds::msg::SoftwareVersion>, 
	public PublishROS2Msg<ros2::msg::SoftwareVersion>
{
public:
	DDS_To_ROS2_SoftwareVersion(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::SoftwareVersion>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::SoftwareVersion>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::SoftwareVersion &dds_msg)
		{
			ros2::msg::SoftwareVersion ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.us_msg_data_length = dds_msg.us_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();

			{int size = dds_msg.uc_ver().size();
			for(int i = 0; i < size; ++i)
			{
				ros2_msg.uc_ver[i] = dds_msg.uc_ver()[i];
}}
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // SOFTWARE_VERSION_CONVERT_H