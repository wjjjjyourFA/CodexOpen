#ifndef RADAR_TARGET_CONVERT_H
#define RADAR_TARGET_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/radar_target.hpp"
#include "dds/msg/RadarTarget.hpp"

class ROS2_To_DDS_RadarTarget : public SubscribeROS2Msg<ros2::msg::RadarTarget>,
	public PublishDDSMsg<dds::msg::RadarTarget>
{
public:
	ROS2_To_DDS_RadarTarget(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::RadarTarget>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::RadarTarget>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::RadarTarget &ros2_msg)
		{
			dds::msg::RadarTarget dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.ui_target_num() = ros2_msg.ui_target_num;

			{int size = ros2_msg.st_radar_attribute.size();
			dds_msg.st_radar_attribute().resize(size);
			for(int i = 0; i < size; ++i)
			{
			dds_msg.st_radar_attribute()[i].us_target_id() = ros2_msg.st_radar_attribute[i].us_target_id;
			dds_msg.st_radar_attribute()[i].uc_target_type() = ros2_msg.st_radar_attribute[i].uc_target_type;
			dds_msg.st_radar_attribute()[i].uc_target_flag() = ros2_msg.st_radar_attribute[i].uc_target_flag;
			dds_msg.st_radar_attribute()[i].i_target_x() = ros2_msg.st_radar_attribute[i].i_target_x;
			dds_msg.st_radar_attribute()[i].i_target_y() = ros2_msg.st_radar_attribute[i].i_target_y;
			dds_msg.st_radar_attribute()[i].i_target_z() = ros2_msg.st_radar_attribute[i].i_target_z;
			dds_msg.st_radar_attribute()[i].i_target_vx() = ros2_msg.st_radar_attribute[i].i_target_vx;
			dds_msg.st_radar_attribute()[i].i_target_vy() = ros2_msg.st_radar_attribute[i].i_target_vy;
			dds_msg.st_radar_attribute()[i].us_target_length() = ros2_msg.st_radar_attribute[i].us_target_length;
			dds_msg.st_radar_attribute()[i].us_target_width() = ros2_msg.st_radar_attribute[i].us_target_width;
			dds_msg.st_radar_attribute()[i].us_target_height() = ros2_msg.st_radar_attribute[i].us_target_height;
			}}
			dds_msg.s_reserve1() = ros2_msg.s_reserve1;
			dds_msg.ui_target_heading() = ros2_msg.ui_target_heading;
			dds_msg.s_reserve2() = ros2_msg.s_reserve2;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_RadarTarget : public SubscribeDDSMsg<dds::msg::RadarTarget>, 
	public PublishROS2Msg<ros2::msg::RadarTarget>
{
public:
	DDS_To_ROS2_RadarTarget(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::RadarTarget>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::RadarTarget>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::RadarTarget &dds_msg)
		{
			ros2::msg::RadarTarget ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.ui_target_num = dds_msg.ui_target_num();

			{int size = dds_msg.st_radar_attribute().size();
			ros2_msg.st_radar_attribute.resize(size);
			for(int i = 0; i < size; ++i)
			{
			ros2_msg.st_radar_attribute[i].us_target_id = dds_msg.st_radar_attribute()[i].us_target_id();
			ros2_msg.st_radar_attribute[i].uc_target_type = dds_msg.st_radar_attribute()[i].uc_target_type();
			ros2_msg.st_radar_attribute[i].uc_target_flag = dds_msg.st_radar_attribute()[i].uc_target_flag();
			ros2_msg.st_radar_attribute[i].i_target_x = dds_msg.st_radar_attribute()[i].i_target_x();
			ros2_msg.st_radar_attribute[i].i_target_y = dds_msg.st_radar_attribute()[i].i_target_y();
			ros2_msg.st_radar_attribute[i].i_target_z = dds_msg.st_radar_attribute()[i].i_target_z();
			ros2_msg.st_radar_attribute[i].i_target_vx = dds_msg.st_radar_attribute()[i].i_target_vx();
			ros2_msg.st_radar_attribute[i].i_target_vy = dds_msg.st_radar_attribute()[i].i_target_vy();
			ros2_msg.st_radar_attribute[i].us_target_length = dds_msg.st_radar_attribute()[i].us_target_length();
			ros2_msg.st_radar_attribute[i].us_target_width = dds_msg.st_radar_attribute()[i].us_target_width();
			ros2_msg.st_radar_attribute[i].us_target_height = dds_msg.st_radar_attribute()[i].us_target_height();
}}
			ros2_msg.s_reserve1 = dds_msg.s_reserve1();
			ros2_msg.ui_target_heading = dds_msg.ui_target_heading();
			ros2_msg.s_reserve2 = dds_msg.s_reserve2();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // RADAR_TARGET_CONVERT_H