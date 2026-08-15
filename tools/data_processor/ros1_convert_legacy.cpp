#include "tools/data_processor/ros1_convert_legacy.h"

#include <stdexcept>

#define foreach BOOST_FOREACH

/*  注意检查输入点云的 坐标系 是右前上 还是前左上
 *  并且检查是否在内部做了二次转换
 *  pcl::transformPointCloud(cloud, cloud, transform_mat);
 */

namespace jojo {
namespace tools {

Ros1Convert::Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_  = nh;
  pnh_ = private_nh;
}

Ros1Convert::~Ros1Convert() {}

bool Ros1Convert::Init(std::shared_ptr<jojo::tools::RuntimeConfig> rparam,
                       std::shared_ptr<jojo::tools::InterfaceConfig> iparam) {
  rparam_ = rparam;
  iparam_ = iparam;

  data_processor = std::make_shared<DataProcessor>();
  data_processor->Init(rparam_, iparam_);

  // 这个要提前很久初始化，可能和ROS启动有关
  if (iparam_->b_lidar && iparam_->b_difop) {
    this->InitRslidar();
  }

  // mode = 1 2 3 ==> camera infra star
  ds_camera.resize(iparam_->b_camera, DataStatistic<cv::Mat>("camera", 1));
  ds_infra.resize(iparam_->b_infra, DataStatistic<cv::Mat>("infra", 2));
  ds_star.resize(iparam_->b_star, DataStatistic<cv::Mat>("star", 3));
  ds_radar4d.resize(iparam_->b_radar4d, DataStatistic<uint>("radar4d"));

  return true;
}

void Ros1Convert::Run() {
  sleep(1);
  if (!rparam_->b_save_data) {
    throw std::invalid_argument("Ros1Convert requires b_save_data=true");
  }

  rosbag::Bag bag;
  std::string rosbag_read_path =
      rparam_->rosbag_path + "/" + rparam_->rosbag_name + ".bag";
  std::cout << rosbag_read_path << std::endl;

  bag.open(rosbag_read_path, rosbag::bagmode::Read);
  if (!bag.isOpen()) {
    std::cerr << "Open Bag Wrong " << std::endl;
    return;
  }

  data_processor->Start();

  // 仅限简单消息处理

  std::vector<std::string> topics;
  auto add_topic = [&](const bool flag, const std::string& topic) {
    if (flag) {
      topics.push_back(topic);
    }
  };

  add_topic(iparam_->b_local_pose, iparam_->topic_local_pose_sub);
  add_topic(iparam_->b_global_pose, iparam_->topic_global_pose_sub);
  add_topic(iparam_->b_imu_data, iparam_->topic_imu_data_sub);

  std::cout << "topics size " << topics.size() << std::endl;

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  /* 适用于老代码，C++11 之前。
  // m 在每次迭代时拷贝一份，而不是引用。
  foreach (rosbag::MessageInstance const m, view) {
    const std::string& topic = m.getTopic();
  }
  */
  for (rosbag::MessageInstance const& m : view) {
    Ros1bagParseBase(m);
  }

  data_processor->Stop();

  if (iparam_->b_lidar) {
    std::cout << "lidar" << std::endl;

    if (iparam_->b_difop == 0) {
      std::vector<std::string> topics = {iparam_->topic_lidar_sub};

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      ROS_INFO("\033[1;32m----> start lidar.\033[0m");
      for (rosbag::MessageInstance const& m : view) {
        const std::string& topic = m.getTopic();
        if (topic == iparam_->topic_lidar_sub) {
          auto msg = m.instantiate<sensor_msgs::PointCloud2>();
          this->LidarHandler(msg);
        }
        //
        if (data_processor->b_final) {
          break;
        }
      }
    } else {
      // 没有设计多线程解锁，需要重新单线程
      std::vector<std::string> topics = {iparam_->topic_lidar_ori_sub,
                                         iparam_->topic_lidar_difop_sub};

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      ROS_INFO("\033[1;32m----> start lidar packets.\033[0m");
      for (rosbag::MessageInstance const& m : view) {
        const std::string& topic = m.getTopic();
        this->SendLidarHandler(m, topic);
        //
        if (data_processor->b_final) {
          break;
        }
      }
    }
  }

  // clang-format off
  if(iparam_->b_camera){
    std::cout << "camera" << std::endl;

    this->Ros1bagParseImageWrapper(
      bag, iparam_->b_camera, iparam_->topic_camera_sub, ds_camera, "camera");
  }

  if(iparam_->b_infra){
    this->Ros1bagParseImageWrapper(
      bag, iparam_->b_infra, iparam_->topic_infra_sub, ds_infra, "infra");
  }

  if(iparam_->b_star){
    this->Ros1bagParseImageWrapper(
      bag, iparam_->b_star, iparam_->topic_star_sub, ds_star, "star");
  }
  // clang-format on

  // 数据量较小，所以一块处理。
  if (iparam_->b_radar || iparam_->b_radar4d) {
    std::vector<std::string> topics;
    // 普通 radar
    topics.push_back(iparam_->topic_radar_sub);

    // 4D radar  建立 4D 话题列表 + 快速判定集合
    std::unordered_set<std::string> radar4d_topic_set;
    std::unordered_map<std::string, size_t> radar4d_topic2idx;
    for (size_t i = 0; i < iparam_->topic_radar4d_sub.size(); ++i) {
      const std::string& t = iparam_->topic_radar4d_sub[i];
      topics.push_back(t);  // 加入总体 topics
      radar4d_topic_set.insert(t);  // 用于 O(1) 判定
      radar4d_topic2idx[t] = i;  // 如果想知道第几个雷达
    }

    int num_threads = topics.size();
    boost::asio::thread_pool pool(num_threads);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const& m : view) {
      boost::asio::post(
          pool, [this, m, &radar4d_topic_set, &radar4d_topic2idx]() {
            try {
              std::lock_guard<std::mutex> lock(mtx);

              const std::string& topic = m.getTopic();
              /* ---------- 毫米波雷达 ---------- */
              if (topic == iparam_->topic_radar_sub) {
                if (iparam_->b_radar) {
                  if (ds_radar.num == 0) {
                    ROS_INFO("\033[1;32m----> start radar.\033[0m");
                  }
                  RadarHandler(m);
                }
              }
              /* ---------- 4D 毫米波雷达 ---------- */
              if (radar4d_topic_set.count(topic)) {
                if (iparam_->b_radar4d > 0) {
                  size_t idx = radar4d_topic2idx.at(topic);
                  if (ds_radar4d.at(idx).num == 0) {
                    ROS_INFO("\033[1;32m----> start radar4d_%d.\033[0m", idx);
                  }
                  Radar4DHandler(m, idx);
                }
              }
            } catch (const rosbag::BagFormatException& e) {
              ROS_ERROR("rosbag paser error : %s", e.what());
            }
          });
    }

    pool.join();
  }

  bag.close();
  std::cout << "--- --- stop && end --- --- " << std::endl;

  sleep(2);
  ROS_INFO("----> message global num %d", num_global_pose);
  ROS_INFO("----> message local num %d", num_local_pose);
  ROS_INFO("----> message imu num %d", num_imu_data);

  ROS_INFO("----> message lidar send num %d", num_lidar_send);
  ROS_INFO("----> message lidar recv num %d", num_lidar_recv);

  PrintParserCount(ds_camera, "camera");
  PrintParserCount(ds_infra, "infra");
  PrintParserCount(ds_star, "star");

  ROS_INFO("----> message radar num %d", ds_radar.num);

  PrintParserCount(ds_radar4d, "radar4d");

  ROS_INFO("\033[1;32m----> txt file generated over.\033[0m");

  std::cout.flush();
  ROS_INFO("...");  // ROS日志自带刷新
  return;
}

void Ros1Convert::Ros1bagParseBase(const rosbag::MessageInstance& m) {
  const std::string& topic = m.getTopic();
  if (iparam_->b_local_pose && topic == iparam_->topic_local_pose_sub) {
    if (num_local_pose == 0) {
      ROS_INFO("\033[1;32m----> start local_pose.\033[0m");
    }
    auto msg = m.instantiate<self_state::LocalPose>();
    LocalPoseHandler(msg);
  } else if (iparam_->b_global_pose &&
             topic == iparam_->topic_global_pose_sub) {
    if (num_global_pose == 0) {
      ROS_INFO("\033[1;32m----> start global_pose.\033[0m");
    }
    auto msg = m.instantiate<self_state::GlobalPose>();
    GlobalPoseHandler(msg);
  } else if (iparam_->b_imu_data && topic == iparam_->topic_imu_data_sub) {
    if (num_imu_data == 0) {
      ROS_INFO("\033[1;32m----> start imu_data.\033[0m");
    }
    auto msg = m.instantiate<sensor_msgs::Imu>();
    ImuDataHandler(msg);
  }
  // 可以继续根据其他话题做相应处理
}

void Ros1Convert::LocalPoseHandler(
    const self_state::LocalPose::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    auto& fp_local_pose = data_processor->fp_local_pose;

    // 刘伯凯
    // clang-format off
    /*
    std::cout << "angX=" << msg_ptr->angular_x * 100
              << "angY = "<<(msg_ptr->angular_y*100)
              <<" angZ = "<<(msg_ptr->angular_z*100)<<std::endl;
    fprintf(fp_local_pose, "%lf %u %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d % d\n",
            msg_ptr->local_time,              (uint32_t)msg_ptr->gps_time,
            (int)(msg_ptr->dr_x * 100),       (int)(msg_ptr->dr_y * 100),      (int)(msg_ptr->dr_z * 100),
            (int)(msg_ptr->dr_heading * 100), (int)(msg_ptr->dr_roll * 100),   (int)(msg_ptr->dr_pitch * 100),   
            (int)(msg_ptr->lf_speed * 100),   (int)(msg_ptr->rf_speed * 100), 
            (int)(msg_ptr->lr_speed * 100),   (int)(msg_ptr->rr_speed * 100), 
            (int)(msg_ptr->angular_x * 100),  (int)(msg_ptr->angular_y * 100), (int)(msg_ptr->angular_z * 100),
            (int)(msg_ptr->acc_x * 100),      (int)(msg_ptr->acc_y * 100),     (int)(msg_ptr->acc_z * 100), 
            msg_ptr->steer, msg_ptr->brake,
            msg_ptr->fuel, msg_ptr->trans, 0, 0);
    */
    // clang-format on

    // 霍得磊 newLocalPose
    // clang-format off
    /*
    fprintf(fp_local_pose, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %d %lf %lf %lf %lf\n",
            msg_ptr->local_time,         msg_ptr->gps_time,         msg_ptr->message_num, 
            msg_ptr->dr_x,               msg_ptr->dr_y,             msg_ptr->dr_z, 
            msg_ptr->dr_heading,         msg_ptr->dr_pitch,         msg_ptr->dr_roll,
            msg_ptr->lf_speed,           msg_ptr->rf_speed,
            msg_ptr->lr_speed,           msg_ptr->rr_speed,
            msg_ptr->angular_x,          msg_ptr->angular_y,        msg_ptr->angular_z, 
            msg_ptr->acc_x,              msg_ptr->acc_y,            msg_ptr->acc_z, 
            msg_ptr->steer,              msg_ptr->brake,
            msg_ptr->fuel,               msg_ptr->trans,
            msg_ptr->main_switch_status,
            msg_ptr->points.reserved[0],        msg_ptr->points.reserved[1],
            msg_ptr->points.reserved[2],        msg_ptr->points.reserved[3]);
    */
    // clang-format on

    // UGV2023 ROS_MSG
    // clang-format off
    // cm | 0.01degree | cm/s |  cm/s^2 | 0.01degree/s | 0.01degree/s^2
    /*
    fprintf(fp_local_pose, "%lf %d %d "
            "%d %d %d %d %d %d "
            "%d %d %d %d "
            "%d %d %d %d "
            "%d %d %d %d \n",
            msg_ptr->local_time,                      (int)msg_ptr->UTC_time,                msg_ptr->message_num,
            (int)llround(msg_ptr->dr_x*100),          (int)llround(msg_ptr->dr_y*100),       (int)llround(msg_ptr->dr_z*100), 
            (int)llround(msg_ptr->dr_roll*100),       (int)llround(msg_ptr->dr_pitch*100),   (int)llround(msg_ptr->dr_heading*100), 
            (int)llround(msg_ptr->speed_x*100),       (int)llround(msg_ptr->speed_y*100),    (int)llround(msg_ptr->speed_z*100), 
            (int)llround(msg_ptr->vehicle_speed*100),
            0, 1, -1,
            msg_ptr->driving_direction,   
            0, 0, 0, 0);
    */
    // /* FH CODEX
    fprintf(fp_local_pose, "%lf %u "
            "%d %d %d %d %d %d "
            "%d %d %d %d "
            "%d %d %d "
            "%d %d %d "
            "%d %d %d %d "
            "%d %d \n",
            msg_ptr->local_time,                      (uint32_t)msg_ptr->UTC_time,
            (int)llround(msg_ptr->dr_x*100),          (int)llround(msg_ptr->dr_y*100),       (int)llround(msg_ptr->dr_z*100), 
            (int)llround(msg_ptr->dr_pitch*100),      (int)llround(msg_ptr->dr_roll*100),    (int)llround(msg_ptr->dr_heading*100),

            (int)llround(msg_ptr->speed_x*100),       (int)llround(msg_ptr->speed_y*100),    (int)llround(msg_ptr->speed_z*100), 
            (int)llround(msg_ptr->vehicle_speed*100),

            0, 0, 0,
            0, 0, 0,
            0, 0, 0, msg_ptr->driving_direction,
            0, 0);
    // */
    // clang-format on

    num_local_pose++;
  } else {
    ROS_WARN("the null local_pose message ...");
  }
}

void Ros1Convert::GlobalPoseHandler(
    const self_state::GlobalPose::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    auto& fp_global_pose = data_processor->fp_global_pose;

    // 刘伯凯 1
    // clang-format off
    /*
    fprintf(fp_global_pose, "%lf %u %lf %lf %d %d %d %d %d %d %d %d %d %lf %lf %lf %lf %lf %lf "
                            "%f %f %f %f %f %f %f %f %f "
                            "%d %d "
                            "%d %d %d %d "
                            "%f %f %f %f\n",
            msg_ptr->local_time,                (uint32_t)msg_ptr->gps_time,
            msg_ptr->latitude*1000000,          msg_ptr->longitude*1000000,
            (int)(msg_ptr->gaussX*100),         (int)(msg_ptr->gaussY*100),         (int)(msg_ptr->height*100),
            (int)(msg_ptr->roll*100),           (int)(msg_ptr->pitch*100),          (int)(msg_ptr->azimuth*100),
            (int)(msg_ptr->vNorth*100),         (int)(msg_ptr->vEast*100),          (int)(msg_ptr->vUp*100),
            0.0,      0.0,      0.0,
            0.0,      0.0,      0.0,
            msg_ptr->dev_gaussX,      msg_ptr->dev_gaussY,                           msg_ptr->dev_height,
            msg_ptr->dev_azimuth,     msg_ptr->dev_pitch,                            msg_ptr->dev_roll,
            msg_ptr->dev_vEast,       msg_ptr->dev_vNorth,                           msg_ptr->dev_vUp,
            msg_ptr->ins_status,     (int)msg_ptr->gnss_data.num_satellite_tracked,
            0,      0,      0,      0,
            0.0,    0.0,    0.0,    0.0);
    */
    // clang-format on

    // 刘伯凯 2
    // clang-format off
    /*
    fprintf(fp_global_pose, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %d %d %lf %lf %lf %lf\n",
            msg_ptr->local_time,      msg_ptr->gps_time,        msg_ptr->message_num,
            msg_ptr->gaussX,          msg_ptr->gaussY,          msg_ptr->height,
            msg_ptr->vEast,           msg_ptr->vNorth,          msg_ptr->vUp,
            msg_ptr->azimuth,         msg_ptr->pitch,           msg_ptr->roll,

            msg_ptr->dev_gaussX,      msg_ptr->dev_gaussY,      msg_ptr->dev_height,
            msg_ptr->dev_vEast,       msg_ptr->dev_vNorth,      msg_ptr->dev_vUp,
            msg_ptr->dev_azimuth,     msg_ptr->dev_pitch,       msg_ptr->dev_roll,

            msg_ptr->longitude,       msg_ptr->latitude,

            msg_ptr->ins_status,      msg_ptr->pos_type,
            msg_ptr->gnss_data.sol_status,                      msg_ptr->gnss_data.pos_type,
            (int)msg_ptr->gnss_data.num_satellite_tracked,      (int)msg_ptr->gnss_data.num_satellite_used,
            msg_ptr->points.reserved[0],     msg_ptr->points.reserved[1],     msg_ptr->points.reserved[2],        msg_ptr->points.reserved[3]);
    */
    // clang-format on

    // 霍得磊 newGlobalPose
    // clang-format off
    /*
    fprintf(fp_global_pose, "%lf %lf %d %d %d  %lf %lf %lf %lf %lf %lf %lf %lf %lf  %lf %lf %lf %lf %lf %lf %lf %lf %lf  %lf %lf "
                            "%lf %lf %d %d %d  %lf %lf  %lf %lf %lf "
                            "%lf %lf %d  %lf %lf %lf  %lf %lf %lf "
                            "%lf %lf %lf %lf \n",
            msg_ptr->local_time,      msg_ptr->gps_time,        msg_ptr->message_num,
            msg_ptr->ins_status,      msg_ptr->pos_type,
            msg_ptr->gaussX,          msg_ptr->gaussY,          msg_ptr->height,
            msg_ptr->vNorth,          msg_ptr->vEast,           msg_ptr->vUp,
            msg_ptr->roll,            msg_ptr->pitch,           msg_ptr->azimuth,

            msg_ptr->dev_gaussX,      msg_ptr->dev_gaussY,      msg_ptr->dev_height,
            msg_ptr->dev_vNorth,      msg_ptr->dev_vEast,       msg_ptr->dev_vUp,
            msg_ptr->dev_roll,        msg_ptr->dev_pitch,       msg_ptr->dev_azimuth,

            msg_ptr->latitude,        msg_ptr->longitude,

            msg_ptr->gnss_data.local_time,    msg_ptr->gnss_data.gps_time,          msg_ptr->gnss_data.message_num,
            msg_ptr->gnss_data.sol_status,    msg_ptr->gnss_data.pos_type,
            (int)msg_ptr->gnss_data.num_satellite_tracked,      (int)msg_ptr->gnss_data.num_satellite_used,
            msg_ptr->gnss_data.latitude_gnss, msg_ptr->gnss_data.longitude_gnss,    msg_ptr->gnss_data.height_gnss,

            msg_ptr->raw_imu.local_time,      msg_ptr->raw_imu.gps_time,             msg_ptr->raw_imu.message_num,
            msg_ptr->raw_imu.acc_x,           msg_ptr->raw_imu.acc_y,                msg_ptr->raw_imu.acc_z,
            msg_ptr->raw_imu.angular_x,       msg_ptr->raw_imu.angular_y,            msg_ptr->raw_imu.angular_z,
            msg_ptr->points.reserved[0],      msg_ptr->points.reserved[1],
            msg_ptr->points.reserved[2],      msg_ptr->points.reserved[3]);
    */
    // clang-format on

    // UGV2023 ROS_MSG
    // clang-format off
    /*
    fprintf(fp_global_pose, "%lf %u %d "
            "%lld %lld "
            "%d %d %d %d %d %d "
            "%d %d %d "
            "%d %d %d %d %d %d "
            "%d %d %d "
            "%d %d %d %d %d %d\n",
            msg_ptr->local_time,                      (uint32_t)msg_ptr->UTC_time,              msg_ptr->message_num,
            (int64_t)llround(msg_ptr->latitude*1e7),  (int64_t)llround(msg_ptr->longitude*1e7),
            (int)llround(msg_ptr->gaussX*100),        (int)llround(msg_ptr->gaussY*100),        (int)llround(msg_ptr->height*100),
            (int)llround(msg_ptr->roll*100),          (int)llround(msg_ptr->pitch*100),         (int)llround(msg_ptr->azimuth*100),
            (int)llround(msg_ptr->vEast*100),         (int)llround(msg_ptr->vNorth*100),        (int)llround(msg_ptr->vUp*100),

            (int)llround(msg_ptr->dev_gaussX*1000),   (int)llround(msg_ptr->dev_gaussY*1000),   (int)llround(msg_ptr->dev_height*1000),
            (int)llround(msg_ptr->dev_roll*1000),     (int)llround(msg_ptr->dev_pitch*1000),    (int)llround(msg_ptr->dev_azimuth*1000),
            (int)llround(msg_ptr->dev_vEast*1000),    (int)llround(msg_ptr->dev_vNorth*1000),   (int)llround(msg_ptr->dev_vUp*1000),

            msg_ptr->ins_status,      msg_ptr->pos_type,
            0, 0, 0, 0);
    */
    // /* FH CODEX
    fprintf(fp_global_pose, "%lf %u "
            "%lf %lf "
            "%d %d %d %d %d %d "
            "%d %d %d "
            "%d %d %d "            
            "%d %d %d "
            "%f %f %f %f %f %f "
            "%f %f %f "
            "%d %d %d %d %d %d "
            "%f %f %f %f \n",
            msg_ptr->local_time,                      (uint32_t)msg_ptr->UTC_time,
            msg_ptr->latitude*1e6,                    msg_ptr->longitude*1e6,  // 纬度 经度
            (int)llround(msg_ptr->gaussX*100),        (int)llround(msg_ptr->gaussY*100),        (int)llround(msg_ptr->height*100),
            (int)llround(msg_ptr->roll*100),          (int)llround(msg_ptr->pitch*100),         (int)llround(msg_ptr->azimuth*100),
            (int)llround(msg_ptr->vEast*100),         (int)llround(msg_ptr->vNorth*100),         (int)llround(msg_ptr->vUp*100),
            0, 0, 0,
            0, 0, 0,
            msg_ptr->dev_gaussX,   msg_ptr->dev_gaussY,   msg_ptr->dev_height,
            msg_ptr->dev_roll,     msg_ptr->dev_pitch,    msg_ptr->dev_azimuth,
            msg_ptr->dev_vEast,    msg_ptr->dev_vNorth,   msg_ptr->dev_vUp,

            msg_ptr->ins_status,      msg_ptr->pos_type,
            0, 0, 0, 0,
            0.0, 0.0, 0.0, 0.0);
    // */
    // clang-format on

    num_global_pose++;
  } else {
    ROS_WARN("the null global_pose message ...");
  }
}

void Ros1Convert::ImuDataHandler(const sensor_msgs::Imu::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    auto& fp_imu_data = data_processor->fp_imu_data;

    double timestamp = msg_ptr->header.stamp.toSec() * 1000;

    // /* FH CODEX
    // clang-format off
    fprintf(fp_imu_data, "%lf %u %d "
            "%lf %lf %lf %lf %lf %lf\n",
            timestamp,                      0,                              msg_ptr->header.seq,
            msg_ptr->angular_velocity.x,    msg_ptr->angular_velocity.y,    msg_ptr->angular_velocity.z,
            msg_ptr->linear_acceleration.x, msg_ptr->linear_acceleration.y, msg_ptr->linear_acceleration.z
          );
    // clang-format on
    // */

    num_imu_data++;
  } else {
    ROS_WARN("the null imu_data message ...");
  }
}

void Ros1Convert::Ros1bagParseImageWrapper(
    const rosbag::Bag& bag, int sensor_count,
    const std::vector<std::string>& topics,
    std::vector<DataStatistic<cv::Mat>>& ds, const std::string& parser_name) {
  for (int i = 0; i < sensor_count; i++) {
    std::vector<std::string> qry = {topics.at(i)};

    rosbag::View view(bag, rosbag::TopicQuery(qry));

    if (ds.at(i).num == 0) {
      ROS_INFO("\033[1;32m----> start %s_%d.\033[0m", parser_name.c_str(),
               i + 1);
    }

    for (rosbag::MessageInstance const& m : view) {
      const std::string& topic = m.getTopic();

      if (rparam_->prepare_data_num != -1 &&
          data_processor->IsEnd(ds.at(i).sampled_index)) {
        break;
      }

      // clang-format off
      if (topic == topics.at(i)) {
        if (iparam_->b_compressed) {
          // 反序列化成 CompressedImage
          auto msg = m.instantiate<sensor_msgs::CompressedImage>();
          CameraCompressedHandler(msg, i, ds.at(i));
        } else {
          // 反序列化成 Image
          auto msg = m.instantiate<sensor_msgs::Image>();
          CameraHandler(msg, i, ds.at(i));
        }
      }
      // clang-format on
    }
  }
}

void Ros1Convert::CameraHandler(const sensor_msgs::Image::ConstPtr& msg_ptr,
                                int& id, DataStatistic<cv::Mat>& ds) {
  if (!msg_ptr) return;

  uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;
  // std::cout << "image msg_time: " << msg_time << std::endl;

  // check the time range
  int64_t diff = 0;
  if (!data_processor->CheckSampledTime(msg_time, ds.sampled_index, diff))
    return;

  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
  // cv::Mat tmp_image = cv_ptr->image.clone();  // 深拷贝，防止数据丢失
  // cv::Mat tmp_image = cv_ptr->image;  // 浅拷贝

  if (rparam_->prepare_data_num == -1) {
    if (abs(diff) < abs(ds.diff)) {
      ds.diff     = diff;
      ds.msg_time = msg_time;
      ds.data     = cv_ptr->image;
      return;
    } else {
      data_processor->ProcessCameraImage(ds.data, ds.msg_time, id, ds.mode);

      ds.sampled_index++;
      ds.diff     = INT64_MAX;
      ds.msg_time = 0;
    }
  } else {
    ds.msg_time = msg_time;
    ds.data     = cv_ptr->image;
    data_processor->ProcessCameraImage(ds.data, ds.msg_time, id, ds.mode);
  }
  // num_image++;
  ds.num++;
}

void Ros1Convert::CameraCompressedHandler(
    const sensor_msgs::CompressedImage::ConstPtr& msg_ptr, int& id,
    DataStatistic<cv::Mat>& ds) {
  if (!msg_ptr) return;

  uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;
  // std::cout << "image msg_time: " << msg_time << std::endl;

  // check the time range
  int64_t diff = 0;
  if (!data_processor->CheckSampledTime(msg_time, ds.sampled_index, diff))
    return;

  cv_bridge::CvImagePtr cv_ptr_compressed =
      cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
  // cv::Mat tmp_image = cv_ptr_compressed->image;

  if (rparam_->prepare_data_num != -1) {
    if (abs(diff) < abs(ds.diff)) {
      // 可能还不是最接近的
      ds.diff     = diff;
      ds.msg_time = msg_time;
      ds.data     = cv_ptr_compressed->image;
      return;
    } else {
      // diff 已经开始增大，说明上一帧就是最接近的
      // std::cout << "id : " << id << std::endl;
      // data_processor->ProcessCameraImage(tmp_image, msg_time, id, mode);
      data_processor->ProcessCameraImage(ds.data, ds.msg_time, id, ds.mode);

      // 推进游标
      ds.sampled_index++;
      ds.diff     = INT64_MAX;
      ds.msg_time = 0;
    }
  } else {
    ds.msg_time = msg_time;
    ds.data     = cv_ptr_compressed->image;
    data_processor->ProcessCameraImage(ds.data, ds.msg_time, id, ds.mode);
  }
  // num_image++;
  ds.num++;
}

void Ros1Convert::RadarHandler(const rosbag::MessageInstance& m) {
  auto type = SensorRegistry::Instance().GetRadarType(rparam_->radar_type);
  auto& path_radar = data_processor->path_radar;

  // /* ESR_Radar
  if (type == RadarType::ESR) {
    // /* ESR_Radar  20241008
    auto msg_ptr = m.instantiate<sensor::ESR_Radar_Info>();

    if (msg_ptr != NULL) {
      uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;

      RadarPoint Point;
      std::vector<RadarPoint> PointCloud;

      for (int i = 0; i < msg_ptr->objectNum; i++) {
        Point.id = msg_ptr->objectData[i].targetID;
        // version 1.0
        Point.x          = msg_ptr->objectData[i].x;
        Point.y          = msg_ptr->objectData[i].y;
        Point.range      = msg_ptr->objectData[i].range;
        Point.azimuth    = msg_ptr->objectData[i].angle;
        Point.range_rate = msg_ptr->objectData[i].rangeRate;
        Point.velocity   = msg_ptr->objectData[i].Speed;
        PointCloud.push_back(Point);
      }

      char file_radar[300];
      if (rparam_->use_txt_or_pcd == 0) {
        sprintf(file_radar, "%s/%ld.txt", path_radar.c_str(), msg_time);
        FILE* fp_radar;
        fp_radar = fopen(file_radar, "w");
        if (fp_radar == NULL) {
          perror("Radar file create error!");
          return;
        }

        int tmp_id;
        float tmp_x, tmp_y;
        float tmp_range, tmp_angle;
        float tmp_range_rate, tmp_v;
        for (int i = 0; i < PointCloud.size(); i++) {
          tmp_id         = PointCloud[i].id;
          tmp_x          = float(PointCloud[i].x);
          tmp_y          = float(PointCloud[i].y);
          tmp_range      = float(PointCloud[i].range);
          tmp_angle      = float(PointCloud[i].azimuth);
          tmp_range_rate = float(PointCloud[i].range_rate);
          tmp_v          = float(PointCloud[i].velocity);

          // clang-format off
          fprintf(fp_radar, "%d %f %f %f %f %f %f\n",
                  tmp_id, tmp_x, tmp_y, tmp_range, tmp_angle,
                  tmp_range_rate, tmp_v);
          // clang-format on
        }
        fclose(fp_radar);
      }
      PointCloud.clear();
    }
    // */
  }
  // */
  // /* ars_40X
  else if (type == RadarType::ARS408) {
    auto msg_ptr = m.instantiate<ars_40X::ClusterList>();

    if (msg_ptr != NULL) {
      uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;

      RadarPoint Point;
      std::vector<RadarPoint> PointCloud;

      // clang-format off
      for (int i = 0; i < msg_ptr->clusters.size(); i++) {
        Point.id = msg_ptr->clusters[i].id;  // 250
        Point.x  = msg_ptr->clusters[i].position.pose.position.x;
        Point.y  = msg_ptr->clusters[i].position.pose.position.y;
        Point.velocity_x = msg_ptr->clusters[i].relative_velocity.twist.linear.x;
        Point.velocity_y = msg_ptr->clusters[i].relative_velocity.twist.linear.y;
        Point.rcs        = msg_ptr->clusters[i].rcs;
        Point.dyn_prop   = msg_ptr->clusters[i].dynamic_property;
        // std::cout<<" tmp_x "<<Point.x<<std::endl;
        PointCloud.push_back(Point);
      }
      // clang-format on

      char file_radar[300];
      if (rparam_->use_txt_or_pcd == 0) {
        sprintf(file_radar, "%s/%ld.txt", path_radar.c_str(), msg_time);
        FILE* fp_radar;
        fp_radar = fopen(file_radar, "w");
        if (fp_radar == NULL) {
          perror("Radar file create error!");
          return;
        }

        int tmp_x, tmp_y, tmp_vx, tmp_vy, tmp_id;
        int tmp_dynamic_property = 0, tmp_class_type, tmp_prob_of_exist;
        float tmp_rcs;
        // m ==> cm
        for (int i = 0; i < PointCloud.size(); i++) {
          tmp_id               = PointCloud[i].id;
          tmp_x                = int(PointCloud[i].x * 100.0);
          tmp_y                = int(PointCloud[i].y * 100.0);
          tmp_vx               = int(PointCloud[i].velocity_x * 100.0);
          tmp_vy               = int(PointCloud[i].velocity_y * 100.0);
          tmp_rcs              = PointCloud[i].rcs;
          tmp_dynamic_property = PointCloud[i].dyn_prop;
          tmp_prob_of_exist    = PointCloud[i].prob_exist;
          // tmp_class_type = PointCloud[i].class_type;

          // clang-format off
          fprintf(fp_radar, "%d %d %d %d %f %d %d\n", 
                  tmp_x, tmp_y, tmp_vx, tmp_vy, 
                  tmp_rcs, tmp_dynamic_property, tmp_prob_of_exist);
          // clang-format on
        }
        fclose(fp_radar);
      }
      PointCloud.clear();
    }
  }
  // */
  else {
    std::cout << "radar_type set error !!! " << std::endl;
  }
  ds_radar.num++;
}

void Ros1Convert::Radar4DHandler(const rosbag::MessageInstance& m, int id) {
  // 4D毫米波雷达带速度和速度残差
  auto type = SensorRegistry::Instance().GetRadar4dType(rparam_->radar4d_type);
  auto& path_radar4d = data_processor->path_radar4d.at(id);

  // /* ars_548
  if (type == Radar4dType::ARS548) {
    auto msg_ptr = m.instantiate<ars548_msg::DetectionList>();

    if (msg_ptr != NULL) {
      // std::cout<<"start radar 4d "<<std::endl;
      uint64_t msg_time =
          msg_ptr->detection_array[0].header.stamp.toSec() * 1000;

      Radar4DPoint Point;
      std::vector<Radar4DPoint> PointCloud;
      // clang-format off
      for (int i = 0; i < msg_ptr->detection_array.size(); i++) {
        Point.x = msg_ptr->detection_array[i].f_x;
        Point.y = msg_ptr->detection_array[i].f_y;
        Point.z = msg_ptr->detection_array[i].f_z;
        Point.range_rate = msg_ptr->detection_array[i].f_RangeRate;
        Point.range_rate_rms = msg_ptr->detection_array[i].f_RangeRateSTD;
        // Add other fields as necessary.
        PointCloud.push_back(Point);
      }
      // clang-format on
      char file_radar4d[300];
      if (rparam_->use_txt_or_pcd == 0) {
        sprintf(file_radar4d, "%s/%ld.txt", path_radar4d.c_str(), msg_time);
        FILE* fp_radar4d;
        fp_radar4d = fopen(file_radar4d, "w");
        if (fp_radar4d == NULL) {
          perror("Radar4D file create error!");
          return;
        }

        float tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv;
        // m ==> cm
        for (int i = 0; i < PointCloud.size(); i++) {
          tmp_x    = float(PointCloud[i].x * 100.0);
          tmp_y    = float(PointCloud[i].y * 100.0);
          tmp_z    = float(PointCloud[i].z * 100.0);
          tmp_v    = float(PointCloud[i].range_rate * 100.0);
          tmp_stdv = float(PointCloud[i].range_rate_rms * 100.0);

          // clang-format off
          fprintf(fp_radar4d, "%f %f %f %f %f\n", 
                  tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv);
          // clang-format on
        }
        fclose(fp_radar4d);
      }
      PointCloud.clear();
    }

    /* // save sensor_msgs/PointCloud => .pcd
    // PointCloud transfer PointCloud2
    sensor_msgs::PointCloud::ConstPtr msg_ptr =
        m.instantiate<sensor_msgs::PointCloud>();
    sensor_msgs::PointCloud2 cloud2_ptr;
    sensor_msgs::convertPointCloudToPointCloud2(*msg_ptr, cloud2_ptr);
    cloud2_ptr.header.stamp = msg_ptr->header.stamp;
    cloud2_ptr.header.frame_id = msg_ptr->header.frame_id;

    // PointCloud2
    sensor_msgs::PointCloud2 cloud2_ptr =
        m.instantiate<sensor_msgs::PointCloud2>();

    if (cloud2_ptr != NULL) {
      uint64_t msg_time = cloud2_ptr.header.stamp.toSec() * 1000;

      pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(cloud2_ptr, *Cloud);

      char buff[500];
      if (rparam_->use_txt_or_pcd == 1) {
        sprintf(buff, "%s/%ld.pcd", path_radar4d.c_str(), msg_time);
        pcl::io::savePCDFileBinary(buff, *Cloud);
      }
    }
    */
  }
  // */
  // /* hugin_arbe
  else if (type == Radar4dType::ARBE) {
    auto msg_ptr = m.instantiate<sensor_msgs::PointCloud2>();

    if (msg_ptr != NULL) {
      uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000.0;

      hugin::PointCloud PointCloud;

      pcl::fromROSMsg(*msg_ptr, PointCloud.data);
      PointCloud.timestamp = msg_time;

      char file_radar4d[300];
      if (rparam_->use_txt_or_pcd == 0) {
        sprintf(file_radar4d, "%s/%ld.txt", path_radar4d.c_str(), msg_time);
        FILE* fp_radar4d;
        fp_radar4d = fopen(file_radar4d, "w");
        if (fp_radar4d == NULL) {
          perror("Radar4D file create error!");
          return;
        }

        float tmp_x, tmp_y, tmp_z;
        float tmp_range, tmp_azimuth, tmp_elevation;
        float tmp_doppler, tmp_rcs;
        for (int i = 0; i < PointCloud.data.size(); i++) {
          tmp_x         = float(PointCloud.data[i].x);
          tmp_y         = float(PointCloud.data[i].y);
          tmp_z         = float(PointCloud.data[i].z);
          tmp_range     = float(PointCloud.data[i].range);
          tmp_azimuth   = float(PointCloud.data[i].azimuth);
          tmp_elevation = float(PointCloud.data[i].elevation);
          tmp_doppler   = float(PointCloud.data[i].doppler);
          tmp_rcs       = float(PointCloud.data[i].rcs);

          // clang-format off
          fprintf(fp_radar4d, "%f %f %f %f %f %f %f %f\n", 
                  tmp_x, tmp_y, tmp_z, tmp_range, 
                  tmp_azimuth, tmp_elevation, tmp_doppler,
                  tmp_rcs);
          // clang-format on
          fclose(fp_radar4d);
        }
      }
    }
  }
  // */
  else {
    std::cout << "radar4d_type set error !!! " << std::endl;
  }
  ds_radar4d.at(id).num++;
  // std::cout << ds_radar4d.at(id) << std::endl;
}

void Ros1Convert::LidarHandler(
    const sensor_msgs::PointCloud2::ConstPtr& msg_ptr) {
  // std::cout << "PointCloud fields:" << std::endl;
  // for (size_t i = 0; i < msg_ptr->fields.size(); ++i)
  //   std::cout << msg_ptr->fields[i].name << std::endl;

  if (msg_ptr != NULL) {
    uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;

    if (!data_processor->PushSampledTime(msg_time)) return;

    // clang-format off
#if defined(RSLIDAR_OLD)
    // ros1 for intensity int8
    pcl::PointCloud<robosense_ros::PointII>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::PointII>);
#elif defined(RSLIDAR_NEW)
    pcl::PointCloud<robosense_ros::PointIF>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::PointIF>);
#elif defined(VELODYNE)
    pcl::PointCloud<velodyne_ros::PointXYZIR>::Ptr vd_cloud(new pcl::PointCloud<velodyne_ros::PointXYZIR>);
#else
    pcl::PointCloud<robosense_ros::Point>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::Point>);
#endif
    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // clang-format on

#if defined(VELODYNE)
    pcl::fromROSMsg(*msg_ptr, *vd_cloud);
    VdToPcl(vd_cloud, Cloud);
    std::cout << "vd_cloud_ptr convert suc " << std::endl;
#else
    pcl::fromROSMsg(*msg_ptr, *rs_cloud);
    RsToPcl(rs_cloud, Cloud);
#endif

    // 去除 NaN 点
    // std::cout << "Before filter: " << Cloud->size() << " points" << std::endl;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud(
    //     new pcl::PointCloud<pcl::PointXYZI>);
    // std::vector<int> indices;  // 存储有效点的索引
    // pcl::removeNaNFromPointCloud(*Cloud, *FilteredCloud, indices);
    // std::cout << "After filter: " << FilteredCloud->size() << " points" << std::endl;

    // warning
    pcl::transformPointCloud(*Cloud, *Cloud,
                             GetTransMatrix(rparam_->b_lt_none_rt));

    if (rparam_->b_save_data) {
      data_processor->SaveLidarData(Cloud, msg_time);
    }

    num_lidar_recv++;
  }
}

void Ros1Convert::InitRslidar() {
  sem_init(&sem_a, 0, 1);
  // sem_init(&sem_b, 0, 1);

  sub_cloud = nh_.subscribe(iparam_->topic_lidar_sub, 10,
                            &Ros1Convert::RecvLidarHandler, this);

  pub_ori = nh_.advertise<rslidar_msgs::rslidarScan>(
      iparam_->topic_lidar_ori_sub /*rslidar_packets*/, 10);

  pub_difop = nh_.advertise<rslidar_msgs::rslidarPacket>(
      iparam_->topic_lidar_difop_sub /*rslidar_packets_difop*/, 10);

  b_first_pub_difop = true;
}

void Ros1Convert::SendLidarHandler(const rosbag::MessageInstance& m,
                                   const std::string& topic) {
  // std::cout << "topic: " << topic << std::endl;
  if (b_first_pub_difop) {
    /*rslidar_packets_difop*/
    if (topic == std::string(iparam_->topic_lidar_difop_sub)) {
      b_first_pub_difop = false;
      rslidar_msgs::rslidarPacket::ConstPtr difop_ptr =
          m.instantiate<rslidar_msgs::rslidarPacket>();
      if (difop_ptr != NULL) {
        pub_difop.publish(*difop_ptr);
        //  ros::spinOnce();
        std::cout << "pub difop first.\n";
      }
    }
  } else {
    /*rslidar_packets*/
    if (topic == std::string(iparam_->topic_lidar_ori_sub)) {
      rslidar_msgs::rslidarScan::ConstPtr scan_ptr =
          m.instantiate<rslidar_msgs::rslidarScan>();
      if (scan_ptr != NULL) {
        pub_ori.publish(*scan_ptr);
        // std::cout << "pub scan .\n";

        num_lidar_send++;

        // 等待回调处理完成
        sem_wait(&sem_a);
      }
    } else if (topic == std::string(iparam_->topic_lidar_difop_sub)) {
      rslidar_msgs::rslidarPacket::ConstPtr difop_ptr =
          m.instantiate<rslidar_msgs::rslidarPacket>();
      if (difop_ptr != NULL) {
        pub_difop.publish(*difop_ptr);
        // std::cout << "pub difop .\n";
      }
    }
  }
}

void Ros1Convert::RecvLidarHandler(const sensor_msgs::PointCloud2& msg) {
  uint64_t msg_time = msg.header.stamp.toSec() * 1000;
  // std::cout << "lidar msg_time: " << msg_time << std::endl;

  sem_post(&sem_a);

  if (!data_processor->PushSampledTime(msg_time)) return;

  // clang-format off
#if defined(RSLIDAR_OLD)
  // ros1 for intensity int8
  pcl::PointCloud<robosense_ros::PointII>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::PointII>);
#elif defined(RSLIDAR_NEW)
  pcl::PointCloud<robosense_ros::PointIF>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::PointIF>);
#elif defined(VELODYNE)
  pcl::PointCloud<velodyne_ros::PointXYZIR>::Ptr vd_cloud(new pcl::PointCloud<velodyne_ros::PointXYZIR>);
#else
  pcl::PointCloud<robosense_ros::Point>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::Point>);
#endif
  pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // clang-format on

#if defined(VELODYNE)
  pcl::fromROSMsg(msg, *vd_cloud);
  VdToPcl(vd_cloud, Cloud);
  std::cout << "vd_cloud_ptr convert suc " << std::endl;
#else
  pcl::fromROSMsg(msg, *rs_cloud);
  RsToPcl(rs_cloud, Cloud);

  // std::cout << "point_rs_->width: " << rs_cloud->width << std::endl;
  // std::cout << "point_rs_->height: " << rs_cloud->height << std::endl;

  // show_pointcloud_ring(*rs_cloud);
  // show_pointcloud_strategy(*rs_cloud, ColorMode::RING);
  // show_pointcloud_height(*rs_cloud);
#endif

  // 去除 NaN 点
  // std::cout << "Before filter: " << Cloud->size() << " points" << std::endl;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud(
  //     new pcl::PointCloud<pcl::PointXYZI>);
  // std::vector<int> indices;  // 存储有效点的索引
  // pcl::removeNaNFromPointCloud(*Cloud, *FilteredCloud, indices);
  // std::cout << "After filter: " << FilteredCloud->size() << " points" << std::endl;

  // warning
  pcl::transformPointCloud(*Cloud, *Cloud,
                           GetTransMatrix(rparam_->b_lt_none_rt));

  if (rparam_->b_save_data) {
    /*  // 按雷达线束存储
    // for intensity can't use it
    for (int j = 0; j < 1800; j++)
      for (int i = 0; i < 128; i++)
        cloud_tmp.points[j * 128 + laser_sort[i]] =
            cloud.points[j * 128 + i];  // sort the lidar by increasing
                                        // elevation angle

    for (int j = 0; j < 1800; j++)
      for (int i = 0; i < 128; i++)
        cloud.points[i * 1800 + j] = cloud_tmp.points[j * 128 + i];  //
    // transpose the lidar data from angle - by - angle to line - by - line
    */

    data_processor->SaveLidarData(Cloud, msg_time);
  }

  num_lidar_recv++;
}

}  // namespace tools
}  // namespace jojo
