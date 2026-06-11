#include "tools/data_processor/ros1_convert.h"

#define foreach BOOST_FOREACH

/*  注意检查输入点云的 坐标系 是右前上 还是前左上
 *  并且检查是否在内部做了二次转换
 *  pcl::transformPointCloud(cloud, cloud, transform_mat);
 */

namespace jojo {
namespace tools {

Ros1Convert::Ros1Convert() {}

Ros1Convert::~Ros1Convert() {}

void Ros1Convert::Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                       std::shared_ptr<RuntimeConfig> param) {
  nh_    = nh;
  param_ = param;

  data_processor = std::make_shared<DataProcessor>();
  data_processor->Init(param_);

  // 这个要提前很久初始化，可能和ROS启动有关
  if (param_->b_lidar) {
    if (param_->b_difop) {
      this->InitRslidar();
    }
  }

  // mode = 1 2 3 ==> camera infra star
  ds_camera.resize(param_->b_camera, DataStatistic<cv::Mat>("camera", 1));
  ds_infra.resize(param_->b_infra, DataStatistic<cv::Mat>("infra", 2));
  ds_star.resize(param_->b_star, DataStatistic<cv::Mat>("star", 3));
  ds_radar4d.resize(param_->b_radar4d, DataStatistic<uint>("radar4d"));
}

void Ros1Convert::Run() {
  sleep(1);
  if (!param_->b_save_data) {
    std::cout << "b_save_data is false! " << std::endl;
    abort();
  }

  rosbag::Bag bag;
  std::string rosbag_read_path =
      param_->rosbag_path + "/" + param_->rosbag_name + ".bag";
  std::cout << rosbag_read_path << std::endl;

  bag.open(rosbag_read_path, rosbag::bagmode::Read);
  if (!bag.isOpen()) {
    std::cerr << "Open Bag Wrong " << std::endl;
    return;
  }

  data_processor->Start();

  /* ---------------- topic map ---------------- */
  for (size_t i = 0; i < param_->topic_camera_sub.size(); i++)
    camera_topic_map[param_->topic_camera_sub[i]] = i;
  for (size_t i = 0; i < param_->topic_infra_sub.size(); i++)
    infra_topic_map[param_->topic_infra_sub[i]] = i;
  for (size_t i = 0; i < param_->topic_star_sub.size(); i++)
    star_topic_map[param_->topic_star_sub[i]] = i;
  for (size_t i = 0; i < param_->topic_radar4d_sub.size(); i++)
    radar4d_topic_map[param_->topic_radar4d_sub[i]] = i;

  // 仅限简单消息处理
  std::vector<std::string> topics;
  if (param_->b_local_pose) {
    topics.push_back(param_->topic_local_pose_sub);
    ROS_INFO("\033[1;32m----> start local_pose.\033[0m");
  }
  if (param_->b_global_pose) {
    topics.push_back(param_->topic_global_pose_sub);
    ROS_INFO("\033[1;32m----> start global_pose.\033[0m");
  }
  if (param_->b_imu_data) {
    topics.push_back(param_->topic_imu_data_sub);
    ROS_INFO("\033[1;32m----> start imu_data.\033[0m");
  }

  rosbag::View view1(bag, rosbag::TopicQuery(topics));

  /* 适用于老代码，C++11 之前。
  // m 在每次迭代时拷贝一份，而不是引用。
  foreach (rosbag::MessageInstance const m, view) {
    const std::string& topic = m.getTopic();
  }
  */
  for (const rosbag::MessageInstance& m : view1) {
    Ros1bagParseBase(m);
  }

  data_processor->Stop();

  if (param_->b_lidar) {
    ROS_INFO("\033[1;32m----> start lidar.\033[0m");

    if (param_->b_difop == 0) {
      std::vector<std::string> topics = {param_->topic_lidar_sub};

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for (rosbag::MessageInstance const& m : view) {
        const std::string& topic = m.getTopic();
        if (topic == param_->topic_lidar_sub) {
          auto msg = m.instantiate<sensor_msgs::PointCloud2>();
          this->LidarHandler(msg);
        }
        if (data_processor->b_final) {
          break;
        }
      }
    } else {
      // 没有设计多线程解锁，需要重新单线程
      std::vector<std::string> topics = {param_->topic_lidar_ori_sub,
                                         param_->topic_lidar_difop_sub};

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for (rosbag::MessageInstance const& m : view) {
        const std::string& topic = m.getTopic();
        this->SendLidarHandler(m, topic);
        if (data_processor->b_final) {
          break;
        }
      }
    }
  }

  topics.clear();
  if (param_->b_camera) {
    for (size_t i = 0; i < param_->b_camera; ++i) {
      topics.push_back(param_->topic_camera_sub[i]);
      ROS_INFO("\033[1;32m----> start camera_%zu.\033[0m", i + 1);
    }
  }
  if (param_->b_infra) {
    for (size_t i = 0; i < param_->b_infra; ++i) {
      topics.push_back(param_->topic_infra_sub[i]);
      ROS_INFO("\033[1;32m----> start infra_%zu.\033[0m", i + 1);
    }
  }
  if (param_->b_star) {
    for (size_t i = 0; i < param_->b_star; ++i) {
      topics.push_back(param_->topic_star_sub[i]);
      ROS_INFO("\033[1;32m----> start star_%zu.\033[0m", i + 1);
    }
  }

  rosbag::View view2(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : view2) {
    const std::string& topic = m.getTopic();

    if (param_->b_camera) {
      Ros1bagParseImageWrapper(m, topic, camera_topic_map, ds_camera);
    }

    if (param_->b_infra) {
      Ros1bagParseImageWrapper(m, topic, infra_topic_map, ds_infra);
    }

    if (param_->b_star) {
      Ros1bagParseImageWrapper(m, topic, star_topic_map, ds_star);
    }

    // 数据量较小，所以一块处理。
    if (param_->b_radar) {
      RadarHandler(m);
    }

    if (param_->b_radar4d) {
      auto it = radar4d_topic_map.find(topic);
      if (it != radar4d_topic_map.end()) {
        Radar4DHandler(m, it->second);
      }
    }
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
  exit(1);
}

void Ros1Convert::Ros1bagParseBase(const rosbag::MessageInstance& m) {
  const std::string& topic = m.getTopic();
  if (param_->b_local_pose && topic == param_->topic_local_pose_sub) {
    auto msg = m.instantiate<self_state::LocalPose>();
    LocalPoseHandler(msg);
  } else if (param_->b_global_pose && topic == param_->topic_global_pose_sub) {
    auto msg = m.instantiate<self_state::GlobalPose>();
    GlobalPoseHandler(msg);
  } else if (param_->b_imu_data && topic == param_->topic_imu_data_sub) {
    auto msg = m.instantiate<sensor_msgs::Imu>();
    ImuDataHandler(msg);
  }
  // 可以继续根据其他话题做相应处理
}

void Ros1Convert::LocalPoseHandler(
    const self_state::LocalPose::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    auto& fp_local_pose = data_processor->fp_local_pose;

    // CodexOpen ROS_MSG
    // clang-format off
    // cm | 0.01degree | cm/s |  cm/s^2 | 0.01degree/s | 0.01degree/s^2
    fprintf(fp_local_pose, "%lf %u "
            "%d %d %d %d %d %d "
            "%d %d %d %d "
            "%d %d %d "
            "%d %d %d "
            "%d %d %d %d "
            "%d %d \n",
            msg_ptr->local_time,                      (uint32_t)msg_ptr->UTC_time,
            (int)F_ROUND(msg_ptr->dr_x*100),          (int)F_ROUND(msg_ptr->dr_y*100),       (int)F_ROUND(msg_ptr->dr_z*100), 
            (int)F_ROUND(msg_ptr->dr_roll*100),       (int)F_ROUND(msg_ptr->dr_pitch*100),   (int)F_ROUND(msg_ptr->dr_heading*100),

            (int)F_ROUND(msg_ptr->speed_x*100),       (int)F_ROUND(msg_ptr->speed_y*100),    (int)F_ROUND(msg_ptr->speed_z*100), 
            (int)F_ROUND(msg_ptr->vehicle_speed*100),

            0, 0, 0,
            0, 0, 0,
            0, 0, 0, msg_ptr->driving_direction,
            0, 0);
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

    // CodexOpen ROS_MSG
    // clang-format off
    // cm | 0.01degree | cm/s |  cm/s^2 | 0.01degree/s | 0.01degree/s^2
    /*
    fprintf(fp_global_pose, "%lf %u "
            "%lld %lld "
            "%d %d %d %d %d %d "
            "%d %d %d "
            "%d %d %d "            
            "%d %d %d "
            "%d %d %d %d %d %d "
            "%d %d %d "
            "%d %d %d %d %d %d "
            "%f %f %f %f \n",
            msg_ptr->local_time,                      (uint32_t)msg_ptr->UTC_time,
            (int64_t)F_ROUND(msg_ptr->latitude*1e7),  (int64_t)F_ROUND(msg_ptr->longitude*1e7), // 纬度 经度
            (int)F_ROUND(msg_ptr->gaussX*100),        (int)F_ROUND(msg_ptr->gaussY*100),        (int)F_ROUND(msg_ptr->height*100),
            (int)F_ROUND(msg_ptr->azimuth*100),       (int)F_ROUND(msg_ptr->pitch*100),         (int)F_ROUND(msg_ptr->roll*100),
            (int)F_ROUND(msg_ptr->vEast*100),         (int)F_ROUND(msg_ptr->vNorth*100),        (int)F_ROUND(msg_ptr->vUp*100),
            0, 0, 0,
            0, 0, 0,
            (int)F_ROUND(msg_ptr->dev_gaussX*1000),   (int)F_ROUND(msg_ptr->dev_gaussY*1000),   (int)F_ROUND(msg_ptr->dev_height*1000),
            (int)F_ROUND(msg_ptr->dev_azimuth*1000),  (int)F_ROUND(msg_ptr->dev_pitch*1000),    (int)F_ROUND(msg_ptr->dev_roll*1000),
            (int)F_ROUND(msg_ptr->dev_vEast*1000),    (int)F_ROUND(msg_ptr->dev_vNorth*1000),   (int)F_ROUND(msg_ptr->dev_vUp*1000),

            msg_ptr->ins_status,      msg_ptr->pos_type,
            0, 0, 0, 0,
            0.0, 0.0, 0.0, 0.0);
    */
    // /*
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
            msg_ptr->latitude*1e6, /*纬度*/            msg_ptr->longitude*1e6, /*经度*/
            (int)F_ROUND(msg_ptr->gaussX*100),        (int)F_ROUND(msg_ptr->gaussY*100),        (int)F_ROUND(msg_ptr->height*100),
            (int)F_ROUND(msg_ptr->roll*100),          (int)F_ROUND(msg_ptr->pitch*100),         (int)F_ROUND(msg_ptr->azimuth*100),
            (int)F_ROUND(msg_ptr->vEast*100),         (int)F_ROUND(msg_ptr->vNorth*100),        (int)F_ROUND(msg_ptr->vUp*100),
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

    // CodexOpen ROS_MSG
    // clang-format off
    // cm | 0.01degree | cm/s |  cm/s^2 | 0.01degree/s | 0.01degree/s^2
    /*
    fprintf(fp_imu_data, "%lf %lf %d "
            "%d %d %d %d %d %d\n",
            timestamp,                                        0.0,                                              msg_ptr->header.seq,
            (int)F_ROUND(msg_ptr->angular_velocity.x*100),    (int)F_ROUND(msg_ptr->angular_velocity.y*100),    (int)F_ROUND(msg_ptr->angular_velocity.z*100),
            (int)F_ROUND(msg_ptr->linear_acceleration.x*100), (int)F_ROUND(msg_ptr->linear_acceleration.y*100), (int)F_ROUND(msg_ptr->linear_acceleration.z*100)
          );
    */
    // /*
    fprintf(fp_imu_data, "%lf %u %d "
            "%lf %lf %lf %lf %lf %lf\n",
            timestamp,                      0,                              msg_ptr->header.seq,
            msg_ptr->angular_velocity.x,    msg_ptr->angular_velocity.y,    msg_ptr->angular_velocity.z,
            msg_ptr->linear_acceleration.x, msg_ptr->linear_acceleration.y, msg_ptr->linear_acceleration.z
          );
    // */
    // clang-format on

    num_imu_data++;
  } else {
    ROS_WARN("the null imu_data message ...");
  }
}

void Ros1Convert::Ros1bagParseImageWrapper(
    const rosbag::MessageInstance& m, const std::string& topic,
    const std::unordered_map<std::string, int>& topic_map,
    std::vector<DataStatistic<cv::Mat>>& ds_vec) {
  auto it = topic_map.find(topic);
  if (it == topic_map.end()) return;

  size_t idx = it->second;

  if (param_->prepare_data_num != -1 &&
      data_processor->IsEnd(ds_vec[idx].sampled_index)) {
    return;
  }

  ImageTask task;
  task.channel_idx = static_cast<int>(idx);
  task.ds          = &ds_vec[idx];
  task.sensor_mode = ds_vec[idx].mode;

  if (param_->b_compressed) {
    task.type = ImageTask::ImageType::COMPRESSED;
    // 反序列化成 CompressedImage
    task.compressed_msg = m.instantiate<sensor_msgs::CompressedImage>();
    if (!task.compressed_msg) return;
  } else {
    task.type = ImageTask::ImageType::RAW;
    // 反序列化成 Image
    task.raw_msg = m.instantiate<sensor_msgs::Image>();
    if (!task.raw_msg) return;
  }

  ImageWorkerFunc(task, data_processor.get(), param_.get());
}

void Ros1Convert::ImageWorkerFunc(const ImageTask& task, DataProcessor* proc,
                                  RuntimeConfig* param) {
  DataStatistic<cv::Mat>& ds = *task.ds;

  cv_bridge::CvImagePtr cv_ptr;

  try {
    if (task.type == ImageTask::ImageType::RAW) {
      cv_ptr =
          cv_bridge::toCvCopy(task.raw_msg, sensor_msgs::image_encodings::BGR8);
    } else {
      cv_ptr = cv_bridge::toCvCopy(task.compressed_msg,
                                   sensor_msgs::image_encodings::BGR8);
    }
  } catch (const cv_bridge::Exception& e) {
    ROS_WARN("cv_bridge exception in worker: %s", e.what());
    return;
  }

  // 获取时间戳
  uint64_t msg_time = (task.type == ImageTask::ImageType::RAW)
                          ? (task.raw_msg->header.stamp.toSec() * 1000)
                          : (task.compressed_msg->header.stamp.toSec() * 1000);

  // 检查采样时间窗口（CheckSampledTime 内部有锁，可并发调用）
  int64_t diff = 0;
  if (!proc->CheckSampledTime(msg_time, ds.sampled_index, diff)) return;

  if (param->prepare_data_num != -1) {
    // 有采样数量限制：寻找最接近采样点的帧
    if (std::abs(diff) < std::abs(ds.diff)) {
      ds.diff     = diff;
      ds.msg_time = msg_time;
      ds.data     = cv_ptr->image;
      // 还不是最接近的，继续等下一帧
      return;
    } else {
      // 上一帧已是最接近的，处理上一帧
      proc->ProcessCameraImage(ds.data, ds.msg_time, task.channel_idx,
                               task.sensor_mode);
      ds.sampled_index++;
      ds.diff     = INT64_MAX;
      ds.msg_time = 0;
    }
  } else {
    // 无采样限制：直接处理当前帧
    ds.msg_time = msg_time;
    ds.data     = cv_ptr->image;
    proc->ProcessCameraImage(ds.data, ds.msg_time, task.channel_idx,
                             task.sensor_mode);
  }
  ds.num++;
}

void Ros1Convert::RadarHandler(const rosbag::MessageInstance& m) {
  auto type = SensorRegistry::Instance().GetRadarType(param_->radar_type);
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
        Point.x = msg_ptr->objectData[i].x;
        Point.y = msg_ptr->objectData[i].y;
        // version 1.1
        // Point.x = msg_ptr->objectData[i].front_distance;
        // Point.y = msg_ptr->objectData[i].left_distance;

        Point.range      = msg_ptr->objectData[i].range;
        Point.azimuth    = msg_ptr->objectData[i].angle;
        Point.range_rate = msg_ptr->objectData[i].rangeRate;
        Point.velocity   = msg_ptr->objectData[i].Speed;
        PointCloud.push_back(Point);
      }

      char file_radar[300];
      if (param_->use_txt_or_pcd == 0) {
        sprintf(file_radar, "%s/%ld.txt", path_radar.c_str(), msg_time);
        FILE* fp_radar;
        fp_radar = fopen(file_radar, "w");
        if (fp_radar == NULL) {
          perror("Radar file create error!");
        }

        for (int i = 0; i < PointCloud.size(); i++) {
          // clang-format off
          fprintf(fp_radar, "%d %f %f %f %f %f %f\n",
                  PointCloud[i].id,                float(PointCloud[i].x),          float(PointCloud[i].y), 
                  float(PointCloud[i].range),      float(PointCloud[i].azimuth),
                  float(PointCloud[i].range_rate), float(PointCloud[i].velocity));
          // clang-format on
        }
        fclose(fp_radar);
      }
      // PointCloud.clear();
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
      // m ==> cm
      for (int i = 0; i < msg_ptr->clusters.size(); i++) {
        Point.id = msg_ptr->clusters[i].id;  // 250
        Point.x  = msg_ptr->clusters[i].position.pose.position.x * 100.0;
        Point.y  = msg_ptr->clusters[i].position.pose.position.y * 100.0;
        Point.velocity_x = msg_ptr->clusters[i].relative_velocity.twist.linear.x * 100.0;
        Point.velocity_y = msg_ptr->clusters[i].relative_velocity.twist.linear.y * 100.0;
        Point.rcs        = msg_ptr->clusters[i].rcs;
        Point.dyn_prop   = msg_ptr->clusters[i].dynamic_property;
        Point.prob_exist = msg_ptr->clusters[i].prob_of_exist;
        // std::cout<<" tmp_x "<<Point.x<<std::endl;
        PointCloud.push_back(Point);
      }
      // clang-format on

      char file_radar[300];
      if (param_->use_txt_or_pcd == 0) {
        sprintf(file_radar, "%s/%ld.txt", path_radar.c_str(), msg_time);
        FILE* fp_radar;
        fp_radar = fopen(file_radar, "w");
        if (fp_radar == NULL) {
          perror("Radar file create error!");
        }

        for (int i = 0; i < PointCloud.size(); i++) {
          // clang-format off
          fprintf(fp_radar, "%d %d %d %d %f %d %d\n", 
                  int(PointCloud[i].x), int(PointCloud[i].y), 
                  int(PointCloud[i].velocity_x), int(PointCloud[i].velocity_y), 
                  PointCloud[i].rcs, PointCloud[i].dyn_prop, PointCloud[i].prob_exist);
          // clang-format on
        }
        fclose(fp_radar);
      }
      // PointCloud.clear();
    }
  }
  // */
  else {
    std::cout << "radar_type set error !!! " << std::endl;
  }
  ds_radar.num++;
}

void Ros1Convert::Radar4DHandler(const rosbag::MessageInstance& m, int idx) {
  // 4D毫米波雷达带速度和速度残差
  auto type = SensorRegistry::Instance().GetRadar4dType(param_->radar4d_type);
  auto& path_radar4d = data_processor->path_radar4d[idx];

  auto& ds = ds_radar4d[idx];

  // /* ars_548
  if (type == Radar4dType::ARS548) {
    auto msg_ptr = m.instantiate<ars548_msg::DetectionList>();

    if (msg_ptr != NULL) {
      const auto& detections = msg_ptr->detection_array;

      // std::cout<<"start radar 4d "<<std::endl;
      uint64_t msg_time = detections[0].header.stamp.toSec() * 1000;

      Radar4DPoint Point;
      std::vector<Radar4DPoint> PointCloud;
      // clang-format off
      // m ==> cm
      for (int i = 0; i < detections.size(); i++) {
        Point.x = detections[i].f_x * 100.0;
        Point.y = detections[i].f_y * 100.0;
        Point.z = detections[i].f_z * 100.0;
        Point.range_rate = detections[i].f_RangeRate * 100.0;
        Point.range_rate_rms = detections[i].f_RangeRateSTD * 100.0;
        // Add other fields as necessary.
        PointCloud.push_back(Point);
      }
      // clang-format on
      char file_radar4d[300];
      if (param_->use_txt_or_pcd == 0) {
        sprintf(file_radar4d, "%s/%ld.txt", path_radar4d.c_str(), msg_time);
        FILE* fp_radar4d;
        fp_radar4d = fopen(file_radar4d, "w");
        if (fp_radar4d == NULL) {
          perror("Radar4D file create error!");
        }

        for (int i = 0; i < PointCloud.size(); i++) {
          // clang-format off
          fprintf(fp_radar4d, "%f %f %f %f %f\n", 
                  float(PointCloud[i].x), float(PointCloud[i].y), float(PointCloud[i].z), 
                  float(PointCloud[i].range_rate), float(PointCloud[i].range_rate_rms));
          // clang-format on
        }
        fclose(fp_radar4d);
      }
      // PointCloud.clear();
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
      if (param_->use_txt_or_pcd == 1) {
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
      if (param_->use_txt_or_pcd == 0) {
        sprintf(file_radar4d, "%s/%ld.txt", path_radar4d.c_str(), msg_time);
        FILE* fp_radar4d;
        fp_radar4d = fopen(file_radar4d, "w");
        if (fp_radar4d == NULL) {
          perror("Radar4D file create error!");
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
  ds.num++;
  // std::cout << ds.num << std::endl;
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
    // pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // std::vector<int> indices;  // 存储有效点的索引
    // pcl::removeNaNFromPointCloud(*Cloud, *FilteredCloud, indices);
    // std::cout << "After filter: " << FilteredCloud->size() << " points" << std::endl;

    // warning
    pcl::transformPointCloud(*Cloud, *Cloud,
                             GetTransMatrix(param_->b_lt_none_rt));

    if (param_->b_save_data) {
      data_processor->SaveLidarData(Cloud, msg_time);
    }

    num_lidar_recv++;
  }
}

void Ros1Convert::InitRslidar() {
  sem_init(&sem_a, 0, 1);
  // sem_init(&sem_b, 0, 1);

  sub_cloud = nh_.subscribe(param_->topic_lidar_sub, 10,
                            &Ros1Convert::RecvLidarHandler, this);

  pub_ori = nh_.advertise<rslidar_msgs::rslidarScan>(
      param_->topic_lidar_ori_sub /*rslidar_packets*/, 10);

  pub_difop = nh_.advertise<rslidar_msgs::rslidarPacket>(
      param_->topic_lidar_difop_sub /*rslidar_packets_difop*/, 10);

  b_first_pub_difop = true;
}

void Ros1Convert::SendLidarHandler(const rosbag::MessageInstance& m,
                                   const std::string& topic) {
  // std::cout << "topic: " << topic << std::endl;
  if (b_first_pub_difop) {
    /*rslidar_packets_difop*/
    if (topic == std::string(param_->topic_lidar_difop_sub)) {
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
    if (topic == std::string(param_->topic_lidar_ori_sub)) {
      rslidar_msgs::rslidarScan::ConstPtr scan_ptr =
          m.instantiate<rslidar_msgs::rslidarScan>();
      if (scan_ptr != NULL) {
        pub_ori.publish(*scan_ptr);
        // std::cout << "pub scan .\n";

        num_lidar_send++;

        // 等待回调处理完成
        sem_wait(&sem_a);
      }
    } else if (topic == std::string(param_->topic_lidar_difop_sub)) {
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
#else
  pcl::PointCloud<robosense_ros::Point>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::Point>);
#endif
  pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // clang-format on

#if defined(USE_RSLIDAR)
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
                           GetTransMatrix(param_->b_lt_none_rt));

  if (param_->b_save_data) {
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
