#ifndef LOCATION_CONVERT_H
#define LOCATION_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/location.hpp"
#include "dds/msg/Location.hpp"

class ROS2_To_DDS_Location : public SubscribeROS2Msg<ros2::msg::Location>,
	public PublishDDSMsg<dds::msg::Location>
{
public:
	ROS2_To_DDS_Location(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::Location>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::Location>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::Location &ros2_msg)
		{
			dds::msg::Location dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ui_msg_time() = ros2_msg.ui_msg_time;
			dds_msg.i_satel_lon() = ros2_msg.i_satel_lon;
			dds_msg.i_satel_lat() = ros2_msg.i_satel_lat;
			dds_msg.i_state_alt() = ros2_msg.i_state_alt;
			dds_msg.uc_satel_alt() = ros2_msg.uc_satel_alt;
			dds_msg.i_nav_lon() = ros2_msg.i_nav_lon;
			dds_msg.i_nav_lat() = ros2_msg.i_nav_lat;
			dds_msg.i_nav_alt() = ros2_msg.i_nav_alt;
			dds_msg.us_nav_yaw() = ros2_msg.us_nav_yaw;
			dds_msg.s_nav_pitch() = ros2_msg.s_nav_pitch;
			dds_msg.s_nav_roll() = ros2_msg.s_nav_roll;
			dds_msg.s_nav_vx() = ros2_msg.s_nav_vx;
			dds_msg.s_nav_vy() = ros2_msg.s_nav_vy;
			dds_msg.s_nav_vz() = ros2_msg.s_nav_vz;
			dds_msg.s_nav_ax() = ros2_msg.s_nav_ax;
			dds_msg.s_nav_ay() = ros2_msg.s_nav_ay;
			dds_msg.s_nav_az() = ros2_msg.s_nav_az;
			dds_msg.s_nav_wx() = ros2_msg.s_nav_wx;
			dds_msg.s_nav_wy() = ros2_msg.s_nav_wy;
			dds_msg.s_nav_wz() = ros2_msg.s_nav_wz;
			dds_msg.s_nav_wax() = ros2_msg.s_nav_wax;
			dds_msg.s_nav_way() = ros2_msg.s_nav_way;
			dds_msg.s_nav_waz() = ros2_msg.s_nav_waz;
			dds_msg.ui_milage() = ros2_msg.ui_milage;
			dds_msg.ul_satel_time() = ros2_msg.ul_satel_time;
			dds_msg.ui_work_time() = ros2_msg.ui_work_time;
			dds_msg.uc_work_state() = ros2_msg.uc_work_state;
			dds_msg.uc_nav_state() = ros2_msg.uc_nav_state;
			dds_msg.s_reserve() = ros2_msg.s_reserve;
			dds_msg.s_reserve2() = ros2_msg.s_reserve2;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_Location : public SubscribeDDSMsg<dds::msg::Location>, 
	public PublishROS2Msg<ros2::msg::Location>
{
public:
	DDS_To_ROS2_Location(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::Location>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::Location>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::Location &dds_msg)
		{
			ros2::msg::Location ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ui_msg_time = dds_msg.ui_msg_time();
			ros2_msg.i_satel_lon = dds_msg.i_satel_lon();
			ros2_msg.i_satel_lat = dds_msg.i_satel_lat();
			ros2_msg.i_state_alt = dds_msg.i_state_alt();
			ros2_msg.uc_satel_alt = dds_msg.uc_satel_alt();
			ros2_msg.i_nav_lon = dds_msg.i_nav_lon();
			ros2_msg.i_nav_lat = dds_msg.i_nav_lat();
			ros2_msg.i_nav_alt = dds_msg.i_nav_alt();
			ros2_msg.us_nav_yaw = dds_msg.us_nav_yaw();
			ros2_msg.s_nav_pitch = dds_msg.s_nav_pitch();
			ros2_msg.s_nav_roll = dds_msg.s_nav_roll();
			ros2_msg.s_nav_vx = dds_msg.s_nav_vx();
			ros2_msg.s_nav_vy = dds_msg.s_nav_vy();
			ros2_msg.s_nav_vz = dds_msg.s_nav_vz();
			ros2_msg.s_nav_ax = dds_msg.s_nav_ax();
			ros2_msg.s_nav_ay = dds_msg.s_nav_ay();
			ros2_msg.s_nav_az = dds_msg.s_nav_az();
			ros2_msg.s_nav_wx = dds_msg.s_nav_wx();
			ros2_msg.s_nav_wy = dds_msg.s_nav_wy();
			ros2_msg.s_nav_wz = dds_msg.s_nav_wz();
			ros2_msg.s_nav_wax = dds_msg.s_nav_wax();
			ros2_msg.s_nav_way = dds_msg.s_nav_way();
			ros2_msg.s_nav_waz = dds_msg.s_nav_waz();
			ros2_msg.ui_milage = dds_msg.ui_milage();
			ros2_msg.ul_satel_time = dds_msg.ul_satel_time();
			ros2_msg.ui_work_time = dds_msg.ui_work_time();
			ros2_msg.uc_work_state = dds_msg.uc_work_state();
			ros2_msg.uc_nav_state = dds_msg.uc_nav_state();
			ros2_msg.s_reserve = dds_msg.s_reserve();
			ros2_msg.s_reserve2 = dds_msg.s_reserve2();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // LOCATION_CONVERT_H