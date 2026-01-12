#ifndef COMPRESSED_IMAGE_CONVERT_H
#define COMPRESSED_IMAGE_CONVERT_H
#include "header_convert.h"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "idl_msg/dds/msg/CompressedImage.hpp"

typedef sensor_msgs::msg::CompressedImage ros2_compressed_image_type;
typedef dds::msg::CompressedImage dds_compressed_image_type;

class ROS2_To_DDS_CompressedImage : public SubscribeROS2Msg<ros2_compressed_image_type>, public PublishDDSMsg<dds_compressed_image_type>
{
public:
    ROS2_To_DDS_CompressedImage(const std::shared_ptr<rclcpp::Node> &rcl_node,
                               std::string sub_topic_name,
                               uint32_t domain_id,
                               std::string pub_topic_name,
                               bool zcc_flag = false)
        : SubscribeROS2Msg<ros2_compressed_image_type>(rcl_node, sub_topic_name, sub_topic_name),
          PublishDDSMsg<dds_compressed_image_type>(domain_id, pub_topic_name, zcc_flag)
    {
        auto func = [this](const ros2_compressed_image_type &ros2_msg)
        {
            dds_compressed_image_type dds_msg = Transform(ros2_msg);

            this->Publish(dds_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static dds_compressed_image_type Transform(const ros2_compressed_image_type &ros2_msg)
    {
        dds_compressed_image_type dds_msg;
        dds_msg.header() = ROS2_To_DDS_Header::Transform(ros2_msg.header);
        dds_msg.format() = ros2_msg.format;
        dds_msg.data() = std::move(ros2_msg.data);

        return dds_msg;
    }
};

class DDS_To_ROS2_CompressedImage : public SubscribeDDSMsg<dds_compressed_image_type>, public PublishROS2Msg<ros2_compressed_image_type>
{
public:
    DDS_To_ROS2_CompressedImage(const std::shared_ptr<rclcpp::Node> &rcl_node,
                               std::string sub_topic_name,
                               uint32_t domain_id,
                               std::string pub_topic_name,
                               bool zcc_flag = false)
        : SubscribeDDSMsg<dds_compressed_image_type>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
          PublishROS2Msg<ros2_compressed_image_type>(rcl_node, pub_topic_name)
    {
        auto func = [this](const dds_compressed_image_type &dds_msg)
        {
            ros2_compressed_image_type ros2_msg = Transform(dds_msg);

            this->Publish(ros2_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static ros2_compressed_image_type Transform(const dds_compressed_image_type &dds_msg)
    {
        ros2_compressed_image_type ros2_msg;
        ros2_msg.header = DDS_To_ROS2_Header::Transform(dds_msg.header());
        ros2_msg.format = dds_msg.format();
        ros2_msg.data = std::move(dds_msg.data());

        return ros2_msg;
    }
};
#endif // COMPRESSED_IMAGE_CONVERT_H
