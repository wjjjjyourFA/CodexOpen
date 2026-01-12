#ifndef SENSOR_FUSION_LOCALIZATION_CONVERT_H
#define SENSOR_FUSION_LOCALIZATION_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/sensor_fusion_localization.hpp"
#include "dds/msg/SensorFusionLocalization.hpp"

class ROS2_To_DDS_SensorFusionLocalization : public SubscribeROS2Msg<ros2::msg::SensorFusionLocalization>,
	public PublishDDSMsg<dds::msg::SensorFusionLocalization>
{
public:
	ROS2_To_DDS_SensorFusionLocalization(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::SensorFusionLocalization>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::SensorFusionLocalization>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::SensorFusionLocalization &ros2_msg)
		{
			dds::msg::SensorFusionLocalization dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.i_fuse_lon() = ros2_msg.i_fuse_lon;
			dds_msg.i_fuse_lat() = ros2_msg.i_fuse_lat;
			dds_msg.i_fuse_alt() = ros2_msg.i_fuse_alt;
			dds_msg.us_fuse_yaw() = ros2_msg.us_fuse_yaw;
			dds_msg.s_fuse_pitch() = ros2_msg.s_fuse_pitch;
			dds_msg.s_fuse_roll() = ros2_msg.s_fuse_roll;
			dds_msg.s_fuse_vx() = ros2_msg.s_fuse_vx;
			dds_msg.s_fuse_vy() = ros2_msg.s_fuse_vy;
			dds_msg.s_fuse_vz() = ros2_msg.s_fuse_vz;
			dds_msg.s_fuse_wx() = ros2_msg.s_fuse_wx;
			dds_msg.s_fuse_wy() = ros2_msg.s_fuse_wy;
			dds_msg.s_fuse_wz() = ros2_msg.s_fuse_wz;
			dds_msg.s_reserve1() = ros2_msg.s_reserve1;
			dds_msg.s_reserve2() = ros2_msg.s_reserve2;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_SensorFusionLocalization : public SubscribeDDSMsg<dds::msg::SensorFusionLocalization>, 
	public PublishROS2Msg<ros2::msg::SensorFusionLocalization>
{
public:
	DDS_To_ROS2_SensorFusionLocalization(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::SensorFusionLocalization>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::SensorFusionLocalization>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::SensorFusionLocalization &dds_msg)
		{
			ros2::msg::SensorFusionLocalization ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.i_fuse_lon = dds_msg.i_fuse_lon();
			ros2_msg.i_fuse_lat = dds_msg.i_fuse_lat();
			ros2_msg.i_fuse_alt = dds_msg.i_fuse_alt();
			ros2_msg.us_fuse_yaw = dds_msg.us_fuse_yaw();
			ros2_msg.s_fuse_pitch = dds_msg.s_fuse_pitch();
			ros2_msg.s_fuse_roll = dds_msg.s_fuse_roll();
			ros2_msg.s_fuse_vx = dds_msg.s_fuse_vx();
			ros2_msg.s_fuse_vy = dds_msg.s_fuse_vy();
			ros2_msg.s_fuse_vz = dds_msg.s_fuse_vz();
			ros2_msg.s_fuse_wx = dds_msg.s_fuse_wx();
			ros2_msg.s_fuse_wy = dds_msg.s_fuse_wy();
			ros2_msg.s_fuse_wz = dds_msg.s_fuse_wz();
			ros2_msg.s_reserve1 = dds_msg.s_reserve1();
			ros2_msg.s_reserve2 = dds_msg.s_reserve2();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // SENSOR_FUSION_LOCALIZATION_CONVERT_H