#ifndef POINT_FIELD_CONVERT_H
#define POINT_FIELD_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/point_field.hpp"
#include "dds/msg/PointField.hpp"

class ROS2_To_DDS_PointField : public SubscribeROS2Msg<ros2::msg::PointField>,
	public PublishDDSMsg<dds::msg::PointField>
{
public:
	ROS2_To_DDS_PointField(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::PointField>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::PointField>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::PointField &ros2_msg)
		{
			dds::msg::PointField dds_msg;
			dds_msg.name() = ros2_msg.name;
			dds_msg.offset() = ros2_msg.offset;
			dds_msg.datatype() = ros2_msg.datatype;
			dds_msg.count() = ros2_msg.count;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_PointField : public SubscribeDDSMsg<dds::msg::PointField>, 
	public PublishROS2Msg<ros2::msg::PointField>
{
public:
	DDS_To_ROS2_PointField(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::PointField>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::PointField>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::PointField &dds_msg)
		{
			ros2::msg::PointField ros2_msg;
			ros2_msg.name = dds_msg.name();
			ros2_msg.offset = dds_msg.offset();
			ros2_msg.datatype = dds_msg.datatype();
			ros2_msg.count = dds_msg.count();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // POINT_FIELD_CONVERT_H