#ifndef TRACKED_CHASSIS_STATUS_CONVERT_H
#define TRACKED_CHASSIS_STATUS_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/tracked_chassis_status.hpp"
#include "dds/msg/TrackedChassisStatus.hpp"

class ROS2_To_DDS_TrackedChassisStatus : public SubscribeROS2Msg<ros2::msg::TrackedChassisStatus>,
	public PublishDDSMsg<dds::msg::TrackedChassisStatus>
{
public:
	ROS2_To_DDS_TrackedChassisStatus(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::TrackedChassisStatus>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::TrackedChassisStatus>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::TrackedChassisStatus &ros2_msg)
		{
			dds::msg::TrackedChassisStatus dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.uc_mission_state() = ros2_msg.uc_mission_state;
			dds_msg.uc_steer_finish() = ros2_msg.uc_steer_finish;
			dds_msg.i_torque() = ros2_msg.i_torque;
			dds_msg.s_rotate_speed() = ros2_msg.s_rotate_speed;
			dds_msg.uc_work_state() = ros2_msg.uc_work_state;
			dds_msg.us_speed() = ros2_msg.us_speed;
			dds_msg.us_curve() = ros2_msg.us_curve;
			dds_msg.uc_speed_limit() = ros2_msg.uc_speed_limit;
			dds_msg.uc_emerg_stop() = ros2_msg.uc_emerg_stop;
			dds_msg.us_mileage() = ros2_msg.us_mileage;
			dds_msg.us_mileage_total() = ros2_msg.us_mileage_total;
			dds_msg.uc_plat_state() = ros2_msg.uc_plat_state;
			dds_msg.us_reserve1() = ros2_msg.us_reserve1;
			dds_msg.us_reserve2() = ros2_msg.us_reserve2;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_TrackedChassisStatus : public SubscribeDDSMsg<dds::msg::TrackedChassisStatus>, 
	public PublishROS2Msg<ros2::msg::TrackedChassisStatus>
{
public:
	DDS_To_ROS2_TrackedChassisStatus(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::TrackedChassisStatus>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::TrackedChassisStatus>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::TrackedChassisStatus &dds_msg)
		{
			ros2::msg::TrackedChassisStatus ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.uc_mission_state = dds_msg.uc_mission_state();
			ros2_msg.uc_steer_finish = dds_msg.uc_steer_finish();
			ros2_msg.i_torque = dds_msg.i_torque();
			ros2_msg.s_rotate_speed = dds_msg.s_rotate_speed();
			ros2_msg.uc_work_state = dds_msg.uc_work_state();
			ros2_msg.us_speed = dds_msg.us_speed();
			ros2_msg.us_curve = dds_msg.us_curve();
			ros2_msg.uc_speed_limit = dds_msg.uc_speed_limit();
			ros2_msg.uc_emerg_stop = dds_msg.uc_emerg_stop();
			ros2_msg.us_mileage = dds_msg.us_mileage();
			ros2_msg.us_mileage_total = dds_msg.us_mileage_total();
			ros2_msg.uc_plat_state = dds_msg.uc_plat_state();
			ros2_msg.us_reserve1 = dds_msg.us_reserve1();
			ros2_msg.us_reserve2 = dds_msg.us_reserve2();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // TRACKED_CHASSIS_STATUS_CONVERT_H