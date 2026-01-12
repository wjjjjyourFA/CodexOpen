#ifndef WHEELED_CHASSIS_STATUS_CONVERT_H
#define WHEELED_CHASSIS_STATUS_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/wheeled_chassis_status.hpp"
#include "dds/msg/WheeledChassisStatus.hpp"

class ROS2_To_DDS_WheeledChassisStatus : public SubscribeROS2Msg<ros2::msg::WheeledChassisStatus>,
	public PublishDDSMsg<dds::msg::WheeledChassisStatus>
{
public:
	ROS2_To_DDS_WheeledChassisStatus(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::WheeledChassisStatus>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::WheeledChassisStatus>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::WheeledChassisStatus &ros2_msg)
		{
			dds::msg::WheeledChassisStatus dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.uc_pose() = ros2_msg.uc_pose;
			dds_msg.uc_chasis_mode() = ros2_msg.uc_chasis_mode;
			dds_msg.uc_mission_state() = ros2_msg.uc_mission_state;
			dds_msg.uc_work_state() = ros2_msg.uc_work_state;
			dds_msg.uc_steer_finish() = ros2_msg.uc_steer_finish;
			dds_msg.uc_emerg_stop() = ros2_msg.uc_emerg_stop;
			dds_msg.uc_emerg_off() = ros2_msg.uc_emerg_off;
			dds_msg.uc_drive_mode() = ros2_msg.uc_drive_mode;
			dds_msg.uc_alarm() = ros2_msg.uc_alarm;
			dds_msg.uc_chasis_state() = ros2_msg.uc_chasis_state;
			dds_msg.uc_watch_state() = ros2_msg.uc_watch_state;
			dds_msg.uc_park_state() = ros2_msg.uc_park_state;
			dds_msg.us_max_drive_power() = ros2_msg.us_max_drive_power;
			dds_msg.uc_speed_limit() = ros2_msg.uc_speed_limit;
			dds_msg.us_speed() = ros2_msg.us_speed;
			dds_msg.us_curve() = ros2_msg.us_curve;
			dds_msg.s_rotate_speed1() = ros2_msg.s_rotate_speed1;
			dds_msg.s_rotate_speed2() = ros2_msg.s_rotate_speed2;
			dds_msg.s_rotate_speed3() = ros2_msg.s_rotate_speed3;
			dds_msg.s_rotate_speed4() = ros2_msg.s_rotate_speed4;
			dds_msg.s_rotate_speed5() = ros2_msg.s_rotate_speed5;
			dds_msg.s_rotate_speed6() = ros2_msg.s_rotate_speed6;
			dds_msg.s_rotate_speed7() = ros2_msg.s_rotate_speed7;
			dds_msg.s_rotate_speed8() = ros2_msg.s_rotate_speed8;
			dds_msg.s_torque1() = ros2_msg.s_torque1;
			dds_msg.s_torque2() = ros2_msg.s_torque2;
			dds_msg.s_torque3() = ros2_msg.s_torque3;
			dds_msg.s_torque4() = ros2_msg.s_torque4;
			dds_msg.s_torque5() = ros2_msg.s_torque5;
			dds_msg.s_torque6() = ros2_msg.s_torque6;
			dds_msg.s_torque7() = ros2_msg.s_torque7;
			dds_msg.s_torque8() = ros2_msg.s_torque8;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_WheeledChassisStatus : public SubscribeDDSMsg<dds::msg::WheeledChassisStatus>, 
	public PublishROS2Msg<ros2::msg::WheeledChassisStatus>
{
public:
	DDS_To_ROS2_WheeledChassisStatus(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::WheeledChassisStatus>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::WheeledChassisStatus>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::WheeledChassisStatus &dds_msg)
		{
			ros2::msg::WheeledChassisStatus ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.uc_pose = dds_msg.uc_pose();
			ros2_msg.uc_chasis_mode = dds_msg.uc_chasis_mode();
			ros2_msg.uc_mission_state = dds_msg.uc_mission_state();
			ros2_msg.uc_work_state = dds_msg.uc_work_state();
			ros2_msg.uc_steer_finish = dds_msg.uc_steer_finish();
			ros2_msg.uc_emerg_stop = dds_msg.uc_emerg_stop();
			ros2_msg.uc_emerg_off = dds_msg.uc_emerg_off();
			ros2_msg.uc_drive_mode = dds_msg.uc_drive_mode();
			ros2_msg.uc_alarm = dds_msg.uc_alarm();
			ros2_msg.uc_chasis_state = dds_msg.uc_chasis_state();
			ros2_msg.uc_watch_state = dds_msg.uc_watch_state();
			ros2_msg.uc_park_state = dds_msg.uc_park_state();
			ros2_msg.us_max_drive_power = dds_msg.us_max_drive_power();
			ros2_msg.uc_speed_limit = dds_msg.uc_speed_limit();
			ros2_msg.us_speed = dds_msg.us_speed();
			ros2_msg.us_curve = dds_msg.us_curve();
			ros2_msg.s_rotate_speed1 = dds_msg.s_rotate_speed1();
			ros2_msg.s_rotate_speed2 = dds_msg.s_rotate_speed2();
			ros2_msg.s_rotate_speed3 = dds_msg.s_rotate_speed3();
			ros2_msg.s_rotate_speed4 = dds_msg.s_rotate_speed4();
			ros2_msg.s_rotate_speed5 = dds_msg.s_rotate_speed5();
			ros2_msg.s_rotate_speed6 = dds_msg.s_rotate_speed6();
			ros2_msg.s_rotate_speed7 = dds_msg.s_rotate_speed7();
			ros2_msg.s_rotate_speed8 = dds_msg.s_rotate_speed8();
			ros2_msg.s_torque1 = dds_msg.s_torque1();
			ros2_msg.s_torque2 = dds_msg.s_torque2();
			ros2_msg.s_torque3 = dds_msg.s_torque3();
			ros2_msg.s_torque4 = dds_msg.s_torque4();
			ros2_msg.s_torque5 = dds_msg.s_torque5();
			ros2_msg.s_torque6 = dds_msg.s_torque6();
			ros2_msg.s_torque7 = dds_msg.s_torque7();
			ros2_msg.s_torque8 = dds_msg.s_torque8();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // WHEELED_CHASSIS_STATUS_CONVERT_H