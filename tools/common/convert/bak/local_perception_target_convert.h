#ifndef LOCAL_PERCEPTION_TARGET_CONVERT_H
#define LOCAL_PERCEPTION_TARGET_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/local_perception_target.hpp"
#include "dds/msg/LocalPerceptionTarget.hpp"

class ROS2_To_DDS_LocalPerceptionTarget : public SubscribeROS2Msg<ros2::msg::LocalPerceptionTarget>,
	public PublishDDSMsg<dds::msg::LocalPerceptionTarget>
{
public:
	ROS2_To_DDS_LocalPerceptionTarget(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::LocalPerceptionTarget>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::LocalPerceptionTarget>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::LocalPerceptionTarget &ros2_msg)
		{
			dds::msg::LocalPerceptionTarget dds_msg;
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
			dds_msg.st_target_attribute()[i].s_target_pos_x() = ros2_msg.st_target_attribute[i].s_target_pos_x;
			dds_msg.st_target_attribute()[i].s_target_pos_y() = ros2_msg.st_target_attribute[i].s_target_pos_y;
			dds_msg.st_target_attribute()[i].s_target_pos_z() = ros2_msg.st_target_attribute[i].s_target_pos_z;
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

class DDS_To_ROS2_LocalPerceptionTarget : public SubscribeDDSMsg<dds::msg::LocalPerceptionTarget>, 
	public PublishROS2Msg<ros2::msg::LocalPerceptionTarget>
{
public:
	DDS_To_ROS2_LocalPerceptionTarget(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::LocalPerceptionTarget>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::LocalPerceptionTarget>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::LocalPerceptionTarget &dds_msg)
		{
			ros2::msg::LocalPerceptionTarget ros2_msg;
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
			ros2_msg.st_target_attribute[i].s_target_pos_x = dds_msg.st_target_attribute()[i].s_target_pos_x();
			ros2_msg.st_target_attribute[i].s_target_pos_y = dds_msg.st_target_attribute()[i].s_target_pos_y();
			ros2_msg.st_target_attribute[i].s_target_pos_z = dds_msg.st_target_attribute()[i].s_target_pos_z();
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
#endif // LOCAL_PERCEPTION_TARGET_CONVERT_H