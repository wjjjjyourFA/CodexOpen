#ifndef PERCEPTION_TARGET_2D_CONVERT_H
#define PERCEPTION_TARGET_2D_CONVERT_H
#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include <idl_msg/dds/msg/2dPerceptionTarget.hpp>
#include "zw_test/msg/perception_target2_d.hpp"

typedef zw_test::msg::PerceptionTarget2D ros2_perception_Target_2d_type;
typedef dds::msg::TwoDPerceptionTarget dds_perception_Target_2d_type;

class ROS2_To_DDS_PerceptionTarget2D : public SubscribeROS2Msg<ros2_perception_Target_2d_type>, public PublishDDSMsg<dds_perception_Target_2d_type>
{
public:
    ROS2_To_DDS_PerceptionTarget2D(const std::shared_ptr<rclcpp::Node> &rcl_node,
                       std::string sub_topic_name,
                       uint32_t domain_id,
                       std::string pub_topic_name,
                       bool zcc_flag = false,
                       std::string network_interface = "")
        : SubscribeROS2Msg<ros2_perception_Target_2d_type>(rcl_node, sub_topic_name, sub_topic_name),
          PublishDDSMsg<dds_perception_Target_2d_type>(domain_id, pub_topic_name, zcc_flag, network_interface)
    {
        auto func = [this](const ros2_perception_Target_2d_type &ros2_msg)
        {
            dds_perception_Target_2d_type dds_msg = Transform(ros2_msg);

            this->Publish(dds_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static dds_perception_Target_2d_type Transform(const ros2_perception_Target_2d_type &ros2_msg)
    {
        dds_perception_Target_2d_type dds_msg;
        dds_msg.target_num() = ros2_msg.target_num;

        for (const auto &ros2_target : ros2_msg.targets)
        {
            dds::msg::Target dds_target;
            dds_target.category() = ros2_target.category;
            dds_target.confidence() = ros2_target.confidence;
            dds_target.xmin() = ros2_target.xmin;
            dds_target.ymin() = ros2_target.ymin;
            dds_target.xmax() = ros2_target.xmax;
            dds_target.ymax() = ros2_target.ymax;

            dds_msg.targets().push_back(dds_target);
        }

        return dds_msg;
    }
};

class DDS_To_ROS2_PerceptionTarget2D : public SubscribeDDSMsg<dds_perception_Target_2d_type>, public PublishROS2Msg<ros2_perception_Target_2d_type>
{
public:
    DDS_To_ROS2_PerceptionTarget2D(const std::shared_ptr<rclcpp::Node> &rcl_node,
                       std::string sub_topic_name,
                       uint32_t domain_id,
                       std::string pub_topic_name,
                       bool zcc_flag = false,
                       std::string network_interface = "")
        : SubscribeDDSMsg<dds_perception_Target_2d_type>(sub_topic_name, sub_topic_name, domain_id, zcc_flag, network_interface),
          PublishROS2Msg<ros2_perception_Target_2d_type>(rcl_node, pub_topic_name)
    {
        auto func = [this](const dds_perception_Target_2d_type &dds_msg)
        {
            ros2_perception_Target_2d_type ros2_msg = Transform(dds_msg);

            this->Publish(ros2_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static ros2_perception_Target_2d_type Transform(const dds_perception_Target_2d_type &dds_msg)
    {
        ros2_perception_Target_2d_type ros2_msg;
        ros2_msg.target_num = dds_msg.target_num();

        for (const auto &dds_target : dds_msg.targets())
        {
            zw_test::msg::Target2D ros2_target;
            ros2_target.category = dds_target.category();
            ros2_target.confidence = dds_target.confidence();
            ros2_target.xmin = dds_target.xmin();
            ros2_target.ymin = dds_target.ymin();
            ros2_target.xmax = dds_target.xmax();
            ros2_target.ymax = dds_target.ymax();

            ros2_msg.targets.push_back(ros2_target);
        }

        return ros2_msg;
    }
};
#endif // PERCEPTION_TARGET_2D_CONVERT_H
