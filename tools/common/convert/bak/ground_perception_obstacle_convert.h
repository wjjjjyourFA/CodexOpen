#ifndef GROUND_PERCEPTION_OBSTACLE_CONVERT_H
#define GROUND_PERCEPTION_OBSTACLE_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "zw_test/msg/ground_perception_obstacle.hpp"
#include "dds/msg/GroundPerceptionObstacle.hpp"

class ROS2_To_DDS_GroundPerceptionObstacle : public SubscribeROS2Msg<zw_test::msg::GroundPerceptionObstacle>,
	public PublishDDSMsg<dds::msg::GroundPerceptionObstacle>
{
public:
	ROS2_To_DDS_GroundPerceptionObstacle(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
        bool zcc_flag = false,
        std::string network_interface = "")
        : SubscribeROS2Msg<zw_test::msg::GroundPerceptionObstacle>(rcl_node, sub_topic_name, sub_topic_name),
        PublishDDSMsg<dds::msg::GroundPerceptionObstacle>(domain_id, pub_topic_name, zcc_flag, network_interface)
	{
        auto func = [this](const zw_test::msg::GroundPerceptionObstacle &ros2_msg)
		{
			dds::msg::GroundPerceptionObstacle dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;
			dds_msg.us_grid_row_num() = ros2_msg.us_grid_row_num;
            dds_msg.us_grid_column_num() = ros2_msg.us_grid_column_num;
			dds_msg.uc_grid_row_scale() = ros2_msg.uc_grid_row_scale;
			dds_msg.uc_grid_column_scale() = ros2_msg.uc_grid_column_scale;
			dds_msg.us_msg_vehicle_row() = ros2_msg.us_msg_vehicle_row;
			dds_msg.ui_msg_vehicle_fixed_long() = ros2_msg.ui_msg_vehicle_fixed_long;
			dds_msg.ui_msg_vehicle_fixed_lati() = ros2_msg.ui_msg_vehicle_fixed_lati;
			dds_msg.ui_msg_vehicle_fixed_alti() = ros2_msg.ui_msg_vehicle_fixed_alti;
			dds_msg.us_msg_vehicle_fixed_heading() = ros2_msg.us_msg_vehicle_fixed_heading;
			dds_msg.us_msg_vehicle_fixed_pitch() = ros2_msg.us_msg_vehicle_fixed_pitch;
			dds_msg.us_msg_vehicle_fixed_roll() = ros2_msg.us_msg_vehicle_fixed_roll;

            int size = ros2_msg.st_grid_attribute.size();
			dds_msg.st_grid_attribute().resize(size);

			for(int i = 0; i < size; ++i)
			{
                dds_msg.st_grid_attribute()[i].s_occupy_nor_x() = ros2_msg.st_grid_attribute[i].s_occupy_nor_x;
                dds_msg.st_grid_attribute()[i].s_occupy_nor_y() = ros2_msg.st_grid_attribute[i].s_occupy_nor_y;
                dds_msg.st_grid_attribute()[i].s_occupy_nor_z() = ros2_msg.st_grid_attribute[i].s_occupy_nor_z;
            }

			dds_msg.uc_ditch_num() = ros2_msg.uc_ditch_num;
			dds_msg.s_ditch_pos_x1() = ros2_msg.s_ditch_pos_x1;
			dds_msg.s_ditch_pos_y1() = ros2_msg.s_ditch_pos_y1;
			dds_msg.s_ditch_pos_x2() = ros2_msg.s_ditch_pos_x2;
			dds_msg.s_ditch_pos_y2() = ros2_msg.s_ditch_pos_y2;
			dds_msg.us_ditch_width() = ros2_msg.us_ditch_width;
			dds_msg.us_wall_num() = ros2_msg.us_wall_num;
			dds_msg.s_wall_pos_x1() = ros2_msg.s_wall_pos_x1;
			dds_msg.s_wall_pos_y1() = ros2_msg.s_wall_pos_y1;
			dds_msg.s_wall_pos_x2() = ros2_msg.s_wall_pos_x2;
			dds_msg.s_wall_pos_y2() = ros2_msg.s_wall_pos_y2;
			dds_msg.us_wall_height() = ros2_msg.us_wall_height;
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_GroundPerceptionObstacle : public SubscribeDDSMsg<dds::msg::GroundPerceptionObstacle>, 
    public PublishROS2Msg<zw_test::msg::GroundPerceptionObstacle>
{
public:
	DDS_To_ROS2_GroundPerceptionObstacle(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
        bool zcc_flag = false,
        std::string network_interface = "")
        : SubscribeDDSMsg<dds::msg::GroundPerceptionObstacle>(sub_topic_name, sub_topic_name, domain_id, zcc_flag, network_interface),
        PublishROS2Msg<zw_test::msg::GroundPerceptionObstacle>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::GroundPerceptionObstacle &dds_msg)
		{
            zw_test::msg::GroundPerceptionObstacle ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();
			ros2_msg.us_grid_row_num = dds_msg.us_grid_row_num();
            ros2_msg.us_grid_column_num = dds_msg.us_grid_column_num();
			ros2_msg.uc_grid_row_scale = dds_msg.uc_grid_row_scale();
			ros2_msg.uc_grid_column_scale = dds_msg.uc_grid_column_scale();
			ros2_msg.us_msg_vehicle_row = dds_msg.us_msg_vehicle_row();
			ros2_msg.ui_msg_vehicle_fixed_long = dds_msg.ui_msg_vehicle_fixed_long();
			ros2_msg.ui_msg_vehicle_fixed_lati = dds_msg.ui_msg_vehicle_fixed_lati();
			ros2_msg.ui_msg_vehicle_fixed_alti = dds_msg.ui_msg_vehicle_fixed_alti();
			ros2_msg.us_msg_vehicle_fixed_heading = dds_msg.us_msg_vehicle_fixed_heading();
			ros2_msg.us_msg_vehicle_fixed_pitch = dds_msg.us_msg_vehicle_fixed_pitch();
			ros2_msg.us_msg_vehicle_fixed_roll = dds_msg.us_msg_vehicle_fixed_roll();

            int size = dds_msg.st_grid_attribute().size();
			ros2_msg.st_grid_attribute.resize(size);

			for(int i = 0; i < size; ++i)
			{
                ros2_msg.st_grid_attribute[i].s_occupy_nor_x = dds_msg.st_grid_attribute()[i].s_occupy_nor_x();
                ros2_msg.st_grid_attribute[i].s_occupy_nor_y = dds_msg.st_grid_attribute()[i].s_occupy_nor_y();
                ros2_msg.st_grid_attribute[i].s_occupy_nor_z = dds_msg.st_grid_attribute()[i].s_occupy_nor_z();
            }

			ros2_msg.uc_ditch_num = dds_msg.uc_ditch_num();
			ros2_msg.s_ditch_pos_x1 = dds_msg.s_ditch_pos_x1();
			ros2_msg.s_ditch_pos_y1 = dds_msg.s_ditch_pos_y1();
			ros2_msg.s_ditch_pos_x2 = dds_msg.s_ditch_pos_x2();
			ros2_msg.s_ditch_pos_y2 = dds_msg.s_ditch_pos_y2();
			ros2_msg.us_ditch_width = dds_msg.us_ditch_width();
			ros2_msg.us_wall_num = dds_msg.us_wall_num();
			ros2_msg.s_wall_pos_x1 = dds_msg.s_wall_pos_x1();
			ros2_msg.s_wall_pos_y1 = dds_msg.s_wall_pos_y1();
			ros2_msg.s_wall_pos_x2 = dds_msg.s_wall_pos_x2();
			ros2_msg.s_wall_pos_y2 = dds_msg.s_wall_pos_y2();
			ros2_msg.us_wall_height = dds_msg.us_wall_height();
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // GROUND_PERCEPTION_OBSTACLE_CONVERT_H
