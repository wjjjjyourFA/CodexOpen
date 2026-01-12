#ifndef GLOBAL_PERCEPTION_TARGET_CONVERT_H
#define GLOBAL_PERCEPTION_TARGET_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/global_perception_target.hpp"
#include "dds/msg/GlobalPerceptionTarget.hpp"

class ROS2_To_DDS_GlobalPerceptionTarget : public SubscribeROS2Msg<ros2::msg::GlobalPerceptionTarget>,
	public PublishDDSMsg<dds::msg::GlobalPerceptionTarget>
{
public:
	ROS2_To_DDS_GlobalPerceptionTarget(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::GlobalPerceptionTarget>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::GlobalPerceptionTarget>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::GlobalPerceptionTarget &ros2_msg)
		{
			dds::msg::GlobalPerceptionTarget dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.uc_target_num() = ros2_msg.uc_target_num;

			{int size = ros2_msg.st_target_attribute.size();
			dds_msg.st_target_attribute().resize(size);
			for(int i = 0; i < size; ++i)
			{
			dds_msg.st_target_attribute()[i].us_target_id() = ros2_msg.st_target_attribute[i].us_target_id;
			dds_msg.st_target_attribute()[i].us_target_type() = ros2_msg.st_target_attribute[i].us_target_type;
			dds_msg.st_target_attribute()[i].uc_target_conf() = ros2_msg.st_target_attribute[i].uc_target_conf;
			dds_msg.st_target_attribute()[i].s_target_pos_long() = ros2_msg.st_target_attribute[i].s_target_pos_long;
			dds_msg.st_target_attribute()[i].s_target_pos_lati() = ros2_msg.st_target_attribute[i].s_target_pos_lati;
			dds_msg.st_target_attribute()[i].s_target_pos_alti() = ros2_msg.st_target_attribute[i].s_target_pos_alti;
			dds_msg.st_target_attribute()[i].us_target_length() = ros2_msg.st_target_attribute[i].us_target_length;
			dds_msg.st_target_attribute()[i].us_target_width() = ros2_msg.st_target_attribute[i].us_target_width;
			dds_msg.st_target_attribute()[i].us_target_height() = ros2_msg.st_target_attribute[i].us_target_height;
			dds_msg.st_target_attribute()[i].us_target_yaw() = ros2_msg.st_target_attribute[i].us_target_yaw;
			dds_msg.st_target_attribute()[i].us_target_speed() = ros2_msg.st_target_attribute[i].us_target_speed;
			}}
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_GlobalPerceptionTarget : public SubscribeDDSMsg<dds::msg::GlobalPerceptionTarget>, 
	public PublishROS2Msg<ros2::msg::GlobalPerceptionTarget>
{
public:
	DDS_To_ROS2_GlobalPerceptionTarget(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::GlobalPerceptionTarget>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::GlobalPerceptionTarget>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::GlobalPerceptionTarget &dds_msg)
		{
			ros2::msg::GlobalPerceptionTarget ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.uc_target_num = dds_msg.uc_target_num();

			{int size = dds_msg.st_target_attribute().size();
			ros2_msg.st_target_attribute.resize(size);
			for(int i = 0; i < size; ++i)
			{
			ros2_msg.st_target_attribute[i].us_target_id = dds_msg.st_target_attribute()[i].us_target_id();
			ros2_msg.st_target_attribute[i].us_target_type = dds_msg.st_target_attribute()[i].us_target_type();
			ros2_msg.st_target_attribute[i].uc_target_conf = dds_msg.st_target_attribute()[i].uc_target_conf();
			ros2_msg.st_target_attribute[i].s_target_pos_long = dds_msg.st_target_attribute()[i].s_target_pos_long();
			ros2_msg.st_target_attribute[i].s_target_pos_lati = dds_msg.st_target_attribute()[i].s_target_pos_lati();
			ros2_msg.st_target_attribute[i].s_target_pos_alti = dds_msg.st_target_attribute()[i].s_target_pos_alti();
			ros2_msg.st_target_attribute[i].us_target_length = dds_msg.st_target_attribute()[i].us_target_length();
			ros2_msg.st_target_attribute[i].us_target_width = dds_msg.st_target_attribute()[i].us_target_width();
			ros2_msg.st_target_attribute[i].us_target_height = dds_msg.st_target_attribute()[i].us_target_height();
			ros2_msg.st_target_attribute[i].us_target_yaw = dds_msg.st_target_attribute()[i].us_target_yaw();
			ros2_msg.st_target_attribute[i].us_target_speed = dds_msg.st_target_attribute()[i].us_target_speed();
}}
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // GLOBAL_PERCEPTION_TARGET_CONVERT_H