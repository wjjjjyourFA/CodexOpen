#ifndef CAMERA_STATE_CONVERT_H
#define CAMERA_STATE_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/camera_state.hpp"
#include "dds/msg/CameraState.hpp"

class ROS2_To_DDS_CameraState : public SubscribeROS2Msg<ros2::msg::CameraState>,
	public PublishDDSMsg<dds::msg::CameraState>
{
public:
	ROS2_To_DDS_CameraState(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::CameraState>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::CameraState>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::CameraState &ros2_msg)
		{
			dds::msg::CameraState dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;

			{int size = ros2_msg.st_camera_state.size();
			dds_msg.st_camera_state().resize(size);
			for(int i = 0; i < size; ++i)
			{
			dds_msg.st_camera_state()[i].uc_id() = ros2_msg.st_camera_state[i].uc_id;
			dds_msg.st_camera_state()[i].uc_state() = ros2_msg.st_camera_state[i].uc_state;
			dds_msg.st_camera_state()[i].us_err_code() = ros2_msg.st_camera_state[i].us_err_code;
			}}
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_CameraState : public SubscribeDDSMsg<dds::msg::CameraState>, 
	public PublishROS2Msg<ros2::msg::CameraState>
{
public:
	DDS_To_ROS2_CameraState(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::CameraState>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::CameraState>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::CameraState &dds_msg)
		{
			ros2::msg::CameraState ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();

			{int size = dds_msg.st_camera_state().size();
			ros2_msg.st_camera_state.resize(size);
			for(int i = 0; i < size; ++i)
			{
			ros2_msg.st_camera_state[i].uc_id = dds_msg.st_camera_state()[i].uc_id();
			ros2_msg.st_camera_state[i].uc_state = dds_msg.st_camera_state()[i].uc_state();
			ros2_msg.st_camera_state[i].us_err_code = dds_msg.st_camera_state()[i].us_err_code();
}}
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // CAMERA_STATE_CONVERT_H