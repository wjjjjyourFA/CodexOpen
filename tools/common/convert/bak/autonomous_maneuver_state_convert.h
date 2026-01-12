#ifndef AUTONOMOUS_MANEUVER_STATE_CONVERT_H
#define AUTONOMOUS_MANEUVER_STATE_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/autonomous_maneuver_state.hpp"
#include "dds/msg/AutonomousManeuverState.hpp"

class ROS2_To_DDS_AutonomousManeuverState : public SubscribeROS2Msg<ros2::msg::AutonomousManeuverState>,
	public PublishDDSMsg<dds::msg::AutonomousManeuverState>
{
public:
	ROS2_To_DDS_AutonomousManeuverState(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::AutonomousManeuverState>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::AutonomousManeuverState>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::AutonomousManeuverState &ros2_msg)
		{
			dds::msg::AutonomousManeuverState dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.ui_source_sw() = ros2_msg.ui_source_sw;
			dds_msg.uc_heart_beat() = ros2_msg.uc_heart_beat;
			dds_msg.uc_mission_count() = ros2_msg.uc_mission_count;
			dds_msg.uc_percep_state() = ros2_msg.uc_percep_state;
			dds_msg.uc_plan_state() = ros2_msg.uc_plan_state;
			dds_msg.uc_auto_state() = ros2_msg.uc_auto_state;
			dds_msg.uc_mode_type() = ros2_msg.uc_mode_type;
			dds_msg.uc_mission_state() = ros2_msg.uc_mission_state;
			dds_msg.uc_remote_ctrl() = ros2_msg.uc_remote_ctrl;
			dds_msg.uc_down_grade() = ros2_msg.uc_down_grade;
			dds_msg.uc_obstacle_cross() = ros2_msg.uc_obstacle_cross;
			dds_msg.uc_path_mission_ctrl() = ros2_msg.uc_path_mission_ctrl;
			dds_msg.uc_follow_mission_ctrl() = ros2_msg.uc_follow_mission_ctrl;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_AutonomousManeuverState : public SubscribeDDSMsg<dds::msg::AutonomousManeuverState>, 
	public PublishROS2Msg<ros2::msg::AutonomousManeuverState>
{
public:
	DDS_To_ROS2_AutonomousManeuverState(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::AutonomousManeuverState>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::AutonomousManeuverState>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::AutonomousManeuverState &dds_msg)
		{
			ros2::msg::AutonomousManeuverState ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.ui_source_sw = dds_msg.ui_source_sw();
			ros2_msg.uc_heart_beat = dds_msg.uc_heart_beat();
			ros2_msg.uc_mission_count = dds_msg.uc_mission_count();
			ros2_msg.uc_percep_state = dds_msg.uc_percep_state();
			ros2_msg.uc_plan_state = dds_msg.uc_plan_state();
			ros2_msg.uc_auto_state = dds_msg.uc_auto_state();
			ros2_msg.uc_mode_type = dds_msg.uc_mode_type();
			ros2_msg.uc_mission_state = dds_msg.uc_mission_state();
			ros2_msg.uc_remote_ctrl = dds_msg.uc_remote_ctrl();
			ros2_msg.uc_down_grade = dds_msg.uc_down_grade();
			ros2_msg.uc_obstacle_cross = dds_msg.uc_obstacle_cross();
			ros2_msg.uc_path_mission_ctrl = dds_msg.uc_path_mission_ctrl();
			ros2_msg.uc_follow_mission_ctrl = dds_msg.uc_follow_mission_ctrl();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // AUTONOMOUS_MANEUVER_STATE_CONVERT_H