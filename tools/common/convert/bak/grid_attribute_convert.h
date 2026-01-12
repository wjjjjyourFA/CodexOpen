#ifndef GRID_ATTRIBUTE_CONVERT_H
#define GRID_ATTRIBUTE_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/grid_attribute.hpp"
#include "dds/msg/GridAttribute.hpp"

class ROS2_To_DDS_GridAttribute : public SubscribeROS2Msg<ros2::msg::GridAttribute>,
	public PublishDDSMsg<dds::msg::GridAttribute>
{
public:
	ROS2_To_DDS_GridAttribute(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::GridAttribute>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::GridAttribute>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::GridAttribute &ros2_msg)
		{
			dds::msg::GridAttribute dds_msg;
			dds_msg.uc_occupy_attribute() = ros2_msg.uc_occupy_attribute;
			dds_msg.uc_occupy_odds() = ros2_msg.uc_occupy_odds;
			dds_msg.us_height() = ros2_msg.us_height;
			dds_msg.uc_target_type() = ros2_msg.uc_target_type;
			dds_msg.uc_terrain_type() = ros2_msg.uc_terrain_type;
			dds_msg.uc_surface_type() = ros2_msg.uc_surface_type;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_GridAttribute : public SubscribeDDSMsg<dds::msg::GridAttribute>, 
	public PublishROS2Msg<ros2::msg::GridAttribute>
{
public:
	DDS_To_ROS2_GridAttribute(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::GridAttribute>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::GridAttribute>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::GridAttribute &dds_msg)
		{
			ros2::msg::GridAttribute ros2_msg;
			ros2_msg.uc_occupy_attribute = dds_msg.uc_occupy_attribute();
			ros2_msg.uc_occupy_odds = dds_msg.uc_occupy_odds();
			ros2_msg.us_height = dds_msg.us_height();
			ros2_msg.uc_target_type = dds_msg.uc_target_type();
			ros2_msg.uc_terrain_type = dds_msg.uc_terrain_type();
			ros2_msg.uc_surface_type = dds_msg.uc_surface_type();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // GRID_ATTRIBUTE_CONVERT_H