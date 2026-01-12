#ifndef IMAGE_FRAME_CONVERT_H
#define IMAGE_FRAME_CONVERT_H
#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "idl_msg/dds/msg/Image.hpp"
#include "msg_all/msg/image_frame.hpp"
#include "opencv2/imgproc.hpp"
#include "dds_cv_bridge.h"
#include "userconfig.h"

extern UserConfig global_para;

typedef msg_all::msg::ImageFrame ros2_image_frame_type;
typedef dds::msg::Image dds_image_type;

class ROS2_To_DDS_ImageFrame : public SubscribeROS2Msg<ros2_image_frame_type>, public PublishDDSMsg<dds_image_type>
{
public:
    ROS2_To_DDS_ImageFrame(const std::shared_ptr<rclcpp::Node> &rcl_node,
                       std::string sub_topic_name,
                       uint32_t domain_id,
                       std::string pub_topic_name,
                       bool zcc_flag = false,
                       std::string network_interface = "")
        : SubscribeROS2Msg<ros2_image_frame_type>(rcl_node, sub_topic_name, sub_topic_name),
          PublishDDSMsg<dds_image_type>(domain_id, pub_topic_name, zcc_flag, network_interface)
    {
        auto func = [this](const ros2_image_frame_type &ros2_msg)
        {
            std::shared_ptr<image_dds_type> dds_msg_ptr = Transform(ros2_msg);

            this->Publish(*dds_msg_ptr);

            double now = MsgTime::GetSystemTime();
            std::cout << "dt:" << now - pre_time << ", fps:" << 1000.0 / (now - pre_time) << std::endl;
            pre_time = now;
        };

        // 设置回调函数
        this->SetOptionalCallbackFunc(func);
    }

    static std::shared_ptr<image_dds_type> Transform(const ros2_image_frame_type &ros2_msg)
    {
        std::shared_ptr<image_dds_type> dds_msg_ptr;

        int height = ros2_msg.height;
        int width = ros2_msg.width;

        cv::Mat cv_img;
        auto data = ros2_msg.data;
        cv::Mat cv_yuv(height, width, CV_8UC2, data.data());//pFrame为YUV数据地址,另外这里就是用 CV_8UC1非 CV_8UC3.
        cv::cvtColor(cv_yuv, cv_img, cv::COLOR_YUV2BGR_YUY2);
        dds::msg::Header dds_header;
        dds_header.stamp().sec() = ros2_msg.seconds;
        dds_header.stamp().nanosec() = ros2_msg.u_seconds * 1e+6;

        cv::Mat undistort_image;
        remap(cv_img, undistort_image, global_para.map1, global_para.map2, cv::INTER_LINEAR);

        dds_msg_ptr = dds::cv_bridge::CvImage(dds_header, "bgr8", undistort_image).toImageMsg();

//        cv::Mat show_img;
//        cv::resize(undistort_image, show_img, cv::Size(1080,540));
//        cv::imshow("a", show_img);
//        int key = cv::waitKey(1);
//        if (key == 32)
//        {
//            static int count = 1;
//            cv::imwrite("img_" + std::to_string(count++) + ".jpg", undistort_image);
//        }


        return dds_msg_ptr;
    }

private:
    double pre_time;
};

//class DDS_To_ROS2_Image : public SubscribeDDSMsg<dds_image_type>, public PublishROS2Msg<ros2_image_frame_type>
//{
//public:
//    DDS_To_ROS2_Image(const std::shared_ptr<rclcpp::Node> &rcl_node,
//                       std::string sub_topic_name,
//                       uint32_t domain_id,
//                       std::string pub_topic_name,
//                       bool zcc_flag = false)
//        : SubscribeDDSMsg<dds_image_type>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
//          PublishROS2Msg<ros2_image_frame_type>(rcl_node, pub_topic_name)
//    {
//        auto func = [this](const dds_image_type &dds_msg)
//        {
//            ros2_image_frame_type ros2_msg = Transform(dds_msg);

//            this->Publish(ros2_msg);
//        };

//        // 设置回调函数
//        this->SetOptionalCallbackFunc(func);
//    }

//    static ros2_image_frame_type Transform(const dds_image_type &dds_msg)
//    {
//        ros2_image_frame_type ros2_msg;
//        ros2_msg.header = DDS_To_ROS2_Header::Transform(dds_msg.header());
//        ros2_msg.height = dds_msg.height();
//        ros2_msg.width = dds_msg.width();
//        ros2_msg.encoding = dds_msg.encoding();
//        ros2_msg.is_bigendian = dds_msg.is_bigendian();
//        ros2_msg.step = dds_msg.step();
//        ros2_msg.data = std::move(dds_msg.data());

//        return ros2_msg;
//    }
//};


#endif // IMAGE_FRAME_CONVERT_H
