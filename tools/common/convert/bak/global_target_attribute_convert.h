#ifndef GLOBAL_TARGET_ATTRIBUTE_CONVERT_H
#define GLOBAL_TARGET_ATTRIBUTE_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/global_target_attribute.hpp"
#include "dds/msg/GlobalTargetAttribute.hpp"

class ROS2_To_DDS_GlobalTargetAttribute : public SubscribeROS2Msg<ros2::msg::GlobalTargetAttribute>,
	public PublishDDSMsg<dds::msg::GlobalTargetAttribute>
{
public:
	ROS2_To_DDS_GlobalTargetAttribute(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::GlobalTargetAttribute>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::GlobalTargetAttribute>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::GlobalTargetAttribute &ros2_msg)
		{
			dds::msg::GlobalTargetAttribute dds_msg;
			dds_msg.us_target_id() = ros2_msg.us_target_id;
			dds_msg.us_target_type() = ros2_msg.us_target_type;
			dds_msg.uc_target_conf() = ros2_msg.uc_target_conf;
			dds_msg.s_target_pos_long() = ros2_msg.s_target_pos_long;
			dds_msg.s_target_pos_lati() = ros2_msg.s_target_pos_lati;
			dds_msg.s_target_pos_alti() = ros2_msg.s_target_pos_alti;
			dds_msg.us_target_length() = ros2_msg.us_target_length;
			dds_msg.us_target_width() = ros2_msg.us_target_width;
			dds_msg.us_target_height() = ros2_msg.us_target_height;
			dds_msg.us_target_yaw() = ros2_msg.us_target_yaw;
			dds_msg.us_target_speed() = ros2_msg.us_target_speed;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_GlobalTargetAttribute : public SubscribeDDSMsg<dds::msg::GlobalTargetAttribute>, 
	public PublishROS2Msg<ros2::msg::GlobalTargetAttribute>
{
public:
	DDS_To_ROS2_GlobalTargetAttribute(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::GlobalTargetAttribute>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::GlobalTargetAttribute>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::GlobalTargetAttribute &dds_msg)
		{
			ros2::msg::GlobalTargetAttribute ros2_msg;
			ros2_msg.us_target_id = dds_msg.us_target_id();
			ros2_msg.us_target_type = dds_msg.us_target_type();
			ros2_msg.uc_target_conf = dds_msg.uc_target_conf();
			ros2_msg.s_target_pos_long = dds_msg.s_target_pos_long();
			ros2_msg.s_target_pos_lati = dds_msg.s_target_pos_lati();
			ros2_msg.s_target_pos_alti = dds_msg.s_target_pos_alti();
			ros2_msg.us_target_length = dds_msg.us_target_length();
			ros2_msg.us_target_width = dds_msg.us_target_width();
			ros2_msg.us_target_height = dds_msg.us_target_height();
			ros2_msg.us_target_yaw = dds_msg.us_target_yaw();
			ros2_msg.us_target_speed = dds_msg.us_target_speed();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // GLOBAL_TARGET_ATTRIBUTE_CONVERT_H