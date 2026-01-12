#ifndef POINT_CLOUD2_CONVERT_H
#define POINT_CLOUD2_CONVERT_H
#include "header_convert.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "idl_msg/dds/msg/point_cloud/PointCloud2.hpp"

typedef sensor_msgs::msg::PointCloud2 ros2_point_cloud2_type;
typedef dds::msg::PointCloud2 dds_point_cloud2_type;

class ROS2_To_DDS_PointCloud2 : public SubscribeROS2Msg<ros2_point_cloud2_type>, public PublishDDSMsg<dds_point_cloud2_type>
{
public:
    ROS2_To_DDS_PointCloud2(const std::shared_ptr<rclcpp::Node> &rcl_node,
                               std::string sub_topic_name,
                               uint32_t domain_id,
                               std::string pub_topic_name,
                               bool zcc_flag = false,
                               std::string network_interface = "")
        : SubscribeROS2Msg<ros2_point_cloud2_type>(rcl_node, sub_topic_name, sub_topic_name),
          PublishDDSMsg<dds_point_cloud2_type>(domain_id, pub_topic_name, zcc_flag, network_interface)
    {
        auto func = [this](const ros2_point_cloud2_type &ros2_msg)
        {
            dds_point_cloud2_type dds_msg = Transform(ros2_msg);

            this->Publish(dds_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static dds_point_cloud2_type Transform(const ros2_point_cloud2_type &ros2_msg)
    {
        dds_point_cloud2_type dds_msg;
        dds_msg.header() = ROS2_To_DDS_Header::Transform(ros2_msg.header);
        dds_msg.height() = ros2_msg.height;
        dds_msg.width() = ros2_msg.width;

        dds_msg.fields().resize(ros2_msg.fields.size());
        for (int i = 0; i < ros2_msg.fields.size(); i++)
        {
            dds_msg.fields()[i].name() = ros2_msg.fields[i].name;
            dds_msg.fields()[i].offset() = ros2_msg.fields[i].offset;
            dds_msg.fields()[i].datatype() = ros2_msg.fields[i].datatype;
            dds_msg.fields()[i].count() = ros2_msg.fields[i].count;
        }

        dds_msg.is_bigendian() = ros2_msg.is_bigendian;
        dds_msg.point_step() = ros2_msg.point_step;
        dds_msg.row_step() = ros2_msg.row_step;
        std::cout << ros2_msg.row_step << std::endl;
        dds_msg.data() = std::move(ros2_msg.data);
        dds_msg.is_dense() = ros2_msg.is_dense;

        return dds_msg;
    }
};

class DDS_To_ROS2_PointCloud2 : public SubscribeDDSMsg<dds_point_cloud2_type>, public PublishROS2Msg<ros2_point_cloud2_type>
{
public:
    DDS_To_ROS2_PointCloud2(const std::shared_ptr<rclcpp::Node> &rcl_node,
                               std::string sub_topic_name,
                               uint32_t domain_id,
                               std::string pub_topic_name,
                               bool zcc_flag = false,
                               std::string network_interface = "")
        : SubscribeDDSMsg<dds_point_cloud2_type>(sub_topic_name, sub_topic_name, domain_id, zcc_flag, network_interface),
          PublishROS2Msg<ros2_point_cloud2_type>(rcl_node, pub_topic_name)
    {
        auto func = [this](const dds_point_cloud2_type &dds_msg)
        {
            ros2_point_cloud2_type ros2_msg = Transform(dds_msg);

            this->Publish(ros2_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static ros2_point_cloud2_type Transform(const dds_point_cloud2_type &dds_msg)
    {
        ros2_point_cloud2_type ros2_msg;
        ros2_msg.header = DDS_To_ROS2_Header::Transform(dds_msg.header());
        ros2_msg.height = dds_msg.height();
        ros2_msg.width = dds_msg.width();

        ros2_msg.fields.resize(dds_msg.fields().size());
        for (int i = 0; i < dds_msg.fields().size(); i++)
        {
            ros2_msg.fields[i].name = dds_msg.fields()[i].name();
            ros2_msg.fields[i].offset = dds_msg.fields()[i].offset();
            ros2_msg.fields[i].datatype = dds_msg.fields()[i].datatype();
            ros2_msg.fields[i].count = dds_msg.fields()[i].count();
        }

        ros2_msg.is_bigendian = dds_msg.is_bigendian();
        ros2_msg.point_step = dds_msg.point_step();
        ros2_msg.row_step = dds_msg.row_step();
        ros2_msg.data = std::move(dds_msg.data());
        ros2_msg.is_dense = dds_msg.is_dense();

        return ros2_msg;
    }
};
#endif // POINT_CLOUD2_CONVERT_H
