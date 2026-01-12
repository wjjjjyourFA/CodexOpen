#ifndef AUTONOMOUS_TRACKING_ACTION_CONVERT_H
#define AUTONOMOUS_TRACKING_ACTION_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/autonomous_tracking_action.hpp"
#include "dds/msg/AutonomousTrackingAction.hpp"

class ROS2_To_DDS_AutonomousTrackingAction : public SubscribeROS2Msg<ros2::msg::AutonomousTrackingAction>,
	public PublishDDSMsg<dds::msg::AutonomousTrackingAction>
{
public:
	ROS2_To_DDS_AutonomousTrackingAction(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::AutonomousTrackingAction>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::AutonomousTrackingAction>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::AutonomousTrackingAction &ros2_msg)
		{
			dds::msg::AutonomousTrackingAction dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.us_msg_data_length() = ros2_msg.us_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.ui_source_sw() = ros2_msg.ui_source_sw;
			dds_msg.uc_v_tracj_points_num() = ros2_msg.uc_v_tracj_points_num;

			{int size = ros2_msg.ui_v_tracj_point.size();
			dds_msg.ui_v_tracj_point().resize(size);
			for(int i = 0; i < size; ++i)
			{
				dds_msg.ui_v_tracj_point()[i] = ros2_msg.ui_v_tracj_point[i];
			}}
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_AutonomousTrackingAction : public SubscribeDDSMsg<dds::msg::AutonomousTrackingAction>, 
	public PublishROS2Msg<ros2::msg::AutonomousTrackingAction>
{
public:
	DDS_To_ROS2_AutonomousTrackingAction(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::AutonomousTrackingAction>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::AutonomousTrackingAction>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::AutonomousTrackingAction &dds_msg)
		{
			ros2::msg::AutonomousTrackingAction ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.us_msg_data_length = dds_msg.us_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.ui_source_sw = dds_msg.ui_source_sw();
			ros2_msg.uc_v_tracj_points_num = dds_msg.uc_v_tracj_points_num();

			{int size = dds_msg.ui_v_tracj_point().size();
			ros2_msg.ui_v_tracj_point.resize(size);
			for(int i = 0; i < size; ++i)
			{
				ros2_msg.ui_v_tracj_point[i] = dds_msg.ui_v_tracj_point()[i];
}}
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // AUTONOMOUS_TRACKING_ACTION_CONVERT_H