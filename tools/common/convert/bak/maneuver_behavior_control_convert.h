#ifndef MANEUVER_BEHAVIOR_CONTROL_CONVERT_H
#define MANEUVER_BEHAVIOR_CONTROL_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/maneuver_behavior_control.hpp"
#include "dds/msg/ManeuverBehaviorControl.hpp"

class ROS2_To_DDS_ManeuverBehaviorControl : public SubscribeROS2Msg<ros2::msg::ManeuverBehaviorControl>,
	public PublishDDSMsg<dds::msg::ManeuverBehaviorControl>
{
public:
	ROS2_To_DDS_ManeuverBehaviorControl(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::ManeuverBehaviorControl>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::ManeuverBehaviorControl>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::ManeuverBehaviorControl &ros2_msg)
		{
			dds::msg::ManeuverBehaviorControl dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.us_msg_data_lenth() = ros2_msg.us_msg_data_lenth;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.ui_source_swc() = ros2_msg.ui_source_swc;
			dds_msg.uc_mode_type() = ros2_msg.uc_mode_type;
			dds_msg.us_ref_speed() = ros2_msg.us_ref_speed;
			dds_msg.uc_remote_ctrl() = ros2_msg.uc_remote_ctrl;
			dds_msg.uc_down_grade() = ros2_msg.uc_down_grade;
			dds_msg.uc_obstacle_cross() = ros2_msg.uc_obstacle_cross;
			dds_msg.uc_path_mission_ctrl() = ros2_msg.uc_path_mission_ctrl;
			dds_msg.uc_follow_mission_ctrl() = ros2_msg.uc_follow_mission_ctrl;
			dds_msg.uc_cur_work_state() = ros2_msg.uc_cur_work_state;
			dds_msg.uc_reserve() = ros2_msg.uc_reserve;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_ManeuverBehaviorControl : public SubscribeDDSMsg<dds::msg::ManeuverBehaviorControl>, 
	public PublishROS2Msg<ros2::msg::ManeuverBehaviorControl>
{
public:
	DDS_To_ROS2_ManeuverBehaviorControl(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::ManeuverBehaviorControl>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::ManeuverBehaviorControl>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::ManeuverBehaviorControl &dds_msg)
		{
			ros2::msg::ManeuverBehaviorControl ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.us_msg_data_lenth = dds_msg.us_msg_data_lenth();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.ui_source_swc = dds_msg.ui_source_swc();
			ros2_msg.uc_mode_type = dds_msg.uc_mode_type();
			ros2_msg.us_ref_speed = dds_msg.us_ref_speed();
			ros2_msg.uc_remote_ctrl = dds_msg.uc_remote_ctrl();
			ros2_msg.uc_down_grade = dds_msg.uc_down_grade();
			ros2_msg.uc_obstacle_cross = dds_msg.uc_obstacle_cross();
			ros2_msg.uc_path_mission_ctrl = dds_msg.uc_path_mission_ctrl();
			ros2_msg.uc_follow_mission_ctrl = dds_msg.uc_follow_mission_ctrl();
			ros2_msg.uc_cur_work_state = dds_msg.uc_cur_work_state();
			ros2_msg.uc_reserve = dds_msg.uc_reserve();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // MANEUVER_BEHAVIOR_CONTROL_CONVERT_H