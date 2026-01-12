#ifndef IMAGE_CONVERT_H
#define IMAGE_CONVERT_H
#include "header_convert.h"

#include "dds/build/stable_msgs/base/Image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/imgproc.hpp"
#include "tools/common/cv_bridge/dds_cv_bridge.h"
#include "cv_bridge/cv_bridge.h"

typedef sensor_msgs::msg::Image ros2_image_type;
typedef dds::base::msg::Image   dds_image_type;

using namespace jojo::cyber::io;

class Ros2ToDdsImage    : public Ros2ToDdsBase<ros2_header_type, dds_header_type> {
 public:
  using Ros2ToDdsBase::Ros2ToDdsBase;

  
    Ros2ToDdsImage(const std::shared_ptr<rclcpp::Node> &rcl_node,
                       std::string sub_topic_name,
                       uint32_t domain_id,
                       std::string pub_topic_name,
                       bool zcc_flag = false)
        : SubscribeROS2Msg<ros2_image_type>(rcl_node, sub_topic_name, sub_topic_name),
          PublishDDSMsg<dds_image_type>(domain_id, pub_topic_name, zcc_flag)
    {
        auto func = [this](const ros2_image_type &ros2_msg)
        {
            dds_image_type dds_msg = Transform(ros2_msg);

            this->Publish(dds_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static dds_image_type Transform(const ros2_image_type &ros2_msg)
    {
        dds_image_type dds_msg;
//        dds_msg.header() = ROS2_To_DDS_Header::Transform(ros2_msg.header);
//        dds_msg.height() = ros2_msg.height;
//        dds_msg.width() = ros2_msg.width;
//        dds_msg.encoding() = ros2_msg.encoding;
//        dds_msg.is_bigendian() = ros2_msg.is_bigendian;
//        dds_msg.step() = ros2_msg.step;
//        dds_msg.data() = std::move(ros2_msg.data);

        auto ros2_msg_ptr = std::make_shared<ros2_image_type>(ros2_msg);
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(ros2_msg, ros2_msg_ptr, sensor_msgs::image_encodings::BGR8);
        const cv::Mat& cv_img = cv_ptr->image;

//        cv::Mat undistort_image;
//        remap(cv_img, undistort_image, global_para.map1, global_para.map2, cv::INTER_LINEAR);

        dds_msg = *(dds::cv_bridge::CvImage(Ros2ToDdsHeader::Transform(ros2_msg.header), "bgr8", cv_img).toImageMsg());

        return dds_msg;
    }
};

class DdsToRos2Image : public SubscribeDDSMsg<dds_image_type>, public PublishROS2Msg<ros2_image_type>
{
public:
    DdsToRos2Image(const std::shared_ptr<rclcpp::Node> &rcl_node,
                       std::string sub_topic_name,
                       uint32_t domain_id,
                       std::string pub_topic_name,
                       bool zcc_flag = false)
        : SubscribeDDSMsg<dds_image_type>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
          PublishROS2Msg<ros2_image_type>(rcl_node, pub_topic_name)
    {
        auto func = [this](const dds_image_type &dds_msg)
        {
            ros2_image_type ros2_msg = Transform(dds_msg);

            this->Publish(ros2_msg);
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static ros2_image_type Transform(const dds_image_type &dds_msg)
    {
        ros2_image_type ros2_msg;
        ros2_msg.header = DDS_To_ROS2_Header::Transform(dds_msg.header());
        ros2_msg.height = dds_msg.height();
        ros2_msg.width = dds_msg.width();
        ros2_msg.encoding = dds_msg.encoding();
        ros2_msg.is_bigendian = dds_msg.is_bigendian();
        ros2_msg.step = dds_msg.step();
        ros2_msg.data = std::move(dds_msg.data());

        return ros2_msg;
    }
};
#endif // IMAGE_CONVERT_H
