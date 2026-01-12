#ifndef RADAR_ATTRIBUTE_CONVERT_H
#define RADAR_ATTRIBUTE_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/radar_attribute.hpp"
#include "dds/msg/RadarAttribute.hpp"

class ROS2_To_DDS_RadarAttribute : public SubscribeROS2Msg<ros2::msg::RadarAttribute>,
	public PublishDDSMsg<dds::msg::RadarAttribute>
{
public:
	ROS2_To_DDS_RadarAttribute(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::RadarAttribute>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::RadarAttribute>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::RadarAttribute &ros2_msg)
		{
			dds::msg::RadarAttribute dds_msg;
			dds_msg.us_target_id() = ros2_msg.us_target_id;
			dds_msg.uc_target_type() = ros2_msg.uc_target_type;
			dds_msg.uc_target_flag() = ros2_msg.uc_target_flag;
			dds_msg.i_target_x() = ros2_msg.i_target_x;
			dds_msg.i_target_y() = ros2_msg.i_target_y;
			dds_msg.i_target_z() = ros2_msg.i_target_z;
			dds_msg.i_target_vx() = ros2_msg.i_target_vx;
			dds_msg.i_target_vy() = ros2_msg.i_target_vy;
			dds_msg.us_target_length() = ros2_msg.us_target_length;
			dds_msg.us_target_width() = ros2_msg.us_target_width;
			dds_msg.us_target_height() = ros2_msg.us_target_height;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_RadarAttribute : public SubscribeDDSMsg<dds::msg::RadarAttribute>, 
	public PublishROS2Msg<ros2::msg::RadarAttribute>
{
public:
	DDS_To_ROS2_RadarAttribute(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::RadarAttribute>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::RadarAttribute>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::RadarAttribute &dds_msg)
		{
			ros2::msg::RadarAttribute ros2_msg;
			ros2_msg.us_target_id = dds_msg.us_target_id();
			ros2_msg.uc_target_type = dds_msg.uc_target_type();
			ros2_msg.uc_target_flag = dds_msg.uc_target_flag();
			ros2_msg.i_target_x = dds_msg.i_target_x();
			ros2_msg.i_target_y = dds_msg.i_target_y();
			ros2_msg.i_target_z = dds_msg.i_target_z();
			ros2_msg.i_target_vx = dds_msg.i_target_vx();
			ros2_msg.i_target_vy = dds_msg.i_target_vy();
			ros2_msg.us_target_length = dds_msg.us_target_length();
			ros2_msg.us_target_width = dds_msg.us_target_width();
			ros2_msg.us_target_height = dds_msg.us_target_height();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // RADAR_ATTRIBUTE_CONVERT_H