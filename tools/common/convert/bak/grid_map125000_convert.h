#ifndef GRID_MAP_CONVERT_H
#define GRID_MAP_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "zw_test/msg/grid_map125000.hpp"
#include "dds/msg/GridMap125000.hpp"

class ROS2_To_DDS_GridMap125000 : public SubscribeROS2Msg<zw_test::msg::GridMap125000>,
    public PublishDDSMsg<dds::msg::GridMap125000>
{
public:
    ROS2_To_DDS_GridMap125000(const std::shared_ptr<rclcpp::Node> &rcl_node,
        std::string sub_topic_name,
        uint32_t domain_id,
        std::string pub_topic_name,
        bool zcc_flag = false)
        : SubscribeROS2Msg<zw_test::msg::GridMap125000>(rcl_node, sub_topic_name, sub_topic_name),
        PublishDDSMsg<dds::msg::GridMap125000>(domain_id, pub_topic_name, zcc_flag)
    {
        auto func = [this](const zw_test::msg::GridMap125000 &ros2_msg)
        {
            dds::msg::GridMap125000 dds_msg = Transform(ros2_msg);

            this->Publish(dds_msg);
        };

        this->SetOptionalCallbackFunc(func);
    }

    static dds::msg::GridMap125000 Transform(const zw_test::msg::GridMap125000 &ros2_msg)
    {
        dds::msg::GridMap125000 dds_msg;
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
        dds_msg.uc_vhicle_dri_state() = ros2_msg.uc_vhicle_dri_state;
        dds_msg.uc_vehicle_dyna_type() = ros2_msg.uc_vehicle_dyna_type;
        dds_msg.uc_scene_type() = ros2_msg.uc_scene_type;
        dds_msg.uc_slope() = ros2_msg.uc_slope;

        int size = ros2_msg.st_grid_attribute.size();

        for(int i = 0; i < size; ++i)
        {
            dds_msg.st_grid_attribute()[i].uc_occupy_attribute() = ros2_msg.st_grid_attribute[i].uc_occupy_attribute;
            dds_msg.st_grid_attribute()[i].uc_occupy_odds() = ros2_msg.st_grid_attribute[i].uc_occupy_odds;
            dds_msg.st_grid_attribute()[i].us_height() = ros2_msg.st_grid_attribute[i].us_height;
            dds_msg.st_grid_attribute()[i].uc_target_type() = ros2_msg.st_grid_attribute[i].uc_target_type;
            dds_msg.st_grid_attribute()[i].uc_terrain_type() = ros2_msg.st_grid_attribute[i].uc_terrain_type;
            dds_msg.st_grid_attribute()[i].uc_surface_type() = ros2_msg.st_grid_attribute[i].uc_surface_type;
        }

        dds_msg.us_check_sum() = ros2_msg.us_check_sum;

        return dds_msg;
    }
};

class DDS_To_ROS2_GridMap125000 : public SubscribeDDSMsg<dds::msg::GridMap125000>,
    public PublishROS2Msg<zw_test::msg::GridMap125000>
{
public:
    DDS_To_ROS2_GridMap125000(const std::shared_ptr<rclcpp::Node> &rcl_node,
        std::string sub_topic_name,
        uint32_t domain_id,
        std::string pub_topic_name,
        bool zcc_flag = false)
        : SubscribeDDSMsg<dds::msg::GridMap125000>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
        PublishROS2Msg<zw_test::msg::GridMap125000>(rcl_node, pub_topic_name)
    {
        auto func = [this](const dds::msg::GridMap125000 &dds_msg)
        {
            zw_test::msg::GridMap125000 ros2_msg = Transform(dds_msg);

            this->Publish(ros2_msg);
        };

        this->SetOptionalCallbackFunc(func);
    }

    static zw_test::msg::GridMap125000 Transform(const dds::msg::GridMap125000 &dds_msg)
    {
        zw_test::msg::GridMap125000 ros2_msg;
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
        ros2_msg.uc_vhicle_dri_state = dds_msg.uc_vhicle_dri_state();
        ros2_msg.uc_vehicle_dyna_type = dds_msg.uc_vehicle_dyna_type();
        ros2_msg.uc_scene_type = dds_msg.uc_scene_type();
        ros2_msg.uc_slope = dds_msg.uc_slope();

        int size = dds_msg.st_grid_attribute().size();

        for(int i = 0; i < size; ++i)
        {
            ros2_msg.st_grid_attribute[i].uc_occupy_attribute = dds_msg.st_grid_attribute()[i].uc_occupy_attribute();
            ros2_msg.st_grid_attribute[i].uc_occupy_odds = dds_msg.st_grid_attribute()[i].uc_occupy_odds();
            ros2_msg.st_grid_attribute[i].us_height = dds_msg.st_grid_attribute()[i].us_height();
            ros2_msg.st_grid_attribute[i].uc_target_type = dds_msg.st_grid_attribute()[i].uc_target_type();
            ros2_msg.st_grid_attribute[i].uc_terrain_type = dds_msg.st_grid_attribute()[i].uc_terrain_type();
            ros2_msg.st_grid_attribute[i].uc_surface_type = dds_msg.st_grid_attribute()[i].uc_surface_type();
        }

        ros2_msg.us_check_sum = dds_msg.us_check_sum();

        return ros2_msg;
    }
};
#endif // GRID_MAP_CONVERT_H
