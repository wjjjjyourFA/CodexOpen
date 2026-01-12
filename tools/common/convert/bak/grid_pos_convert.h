#ifndef GRID_POS_CONVERT_H
#define GRID_POS_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/grid_pos.hpp"
#include "dds/msg/GridPos.hpp"

class ROS2_To_DDS_GridPos : public SubscribeROS2Msg<ros2::msg::GridPos>,
	public PublishDDSMsg<dds::msg::GridPos>
{
public:
	ROS2_To_DDS_GridPos(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::GridPos>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::GridPos>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::GridPos &ros2_msg)
		{
			dds::msg::GridPos dds_msg;
			dds_msg.s_occupy_nor_x() = ros2_msg.s_occupy_nor_x;
			dds_msg.s_occupy_nor_y() = ros2_msg.s_occupy_nor_y;
			dds_msg.s_occupy_nor_z() = ros2_msg.s_occupy_nor_z;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_GridPos : public SubscribeDDSMsg<dds::msg::GridPos>, 
	public PublishROS2Msg<ros2::msg::GridPos>
{
public:
	DDS_To_ROS2_GridPos(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::GridPos>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::GridPos>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::GridPos &dds_msg)
		{
			ros2::msg::GridPos ros2_msg;
			ros2_msg.s_occupy_nor_x = dds_msg.s_occupy_nor_x();
			ros2_msg.s_occupy_nor_y = dds_msg.s_occupy_nor_y();
			ros2_msg.s_occupy_nor_z = dds_msg.s_occupy_nor_z();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // GRID_POS_CONVERT_H