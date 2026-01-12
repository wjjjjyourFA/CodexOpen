#ifndef FORMATION_POSITION_SHARING_CONVERT_H
#define FORMATION_POSITION_SHARING_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/formation_position_sharing.hpp"
#include "dds/msg/FormationPositionSharing.hpp"

class ROS2_To_DDS_FormationPositionSharing : public SubscribeROS2Msg<ros2::msg::FormationPositionSharing>,
	public PublishDDSMsg<dds::msg::FormationPositionSharing>
{
public:
	ROS2_To_DDS_FormationPositionSharing(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::FormationPositionSharing>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::FormationPositionSharing>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::FormationPositionSharing &ros2_msg)
		{
			dds::msg::FormationPositionSharing dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.us_msg_data_length() = ros2_msg.us_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.uc_formate_info() = ros2_msg.uc_formate_info;
			dds_msg.i_lon_vone() = ros2_msg.i_lon_vone;
			dds_msg.i_lat_vone() = ros2_msg.i_lat_vone;
			dds_msg.i_lon_vtwo() = ros2_msg.i_lon_vtwo;
			dds_msg.i_lat_vtwo() = ros2_msg.i_lat_vtwo;
			dds_msg.i_lon_vthree() = ros2_msg.i_lon_vthree;
			dds_msg.i_lat_vthree() = ros2_msg.i_lat_vthree;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_FormationPositionSharing : public SubscribeDDSMsg<dds::msg::FormationPositionSharing>, 
	public PublishROS2Msg<ros2::msg::FormationPositionSharing>
{
public:
	DDS_To_ROS2_FormationPositionSharing(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::FormationPositionSharing>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::FormationPositionSharing>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::FormationPositionSharing &dds_msg)
		{
			ros2::msg::FormationPositionSharing ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.us_msg_data_length = dds_msg.us_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.uc_formate_info = dds_msg.uc_formate_info();
			ros2_msg.i_lon_vone = dds_msg.i_lon_vone();
			ros2_msg.i_lat_vone = dds_msg.i_lat_vone();
			ros2_msg.i_lon_vtwo = dds_msg.i_lon_vtwo();
			ros2_msg.i_lat_vtwo = dds_msg.i_lat_vtwo();
			ros2_msg.i_lon_vthree = dds_msg.i_lon_vthree();
			ros2_msg.i_lat_vthree = dds_msg.i_lat_vthree();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // FORMATION_POSITION_SHARING_CONVERT_H