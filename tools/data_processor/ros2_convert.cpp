#include "tools/data_processor/ros2_convert.h"

#define foreach BOOST_FOREACH

/*  注意检查输入点云的 坐标系 是右前上 还是前左上
 *  并且检查是否在内部做了二次转换
 *  pcl::transformPointCloud(cloud, cloud, transform_mat);
 */

namespace jojo {
namespace tools {

Ros2Convert::Ros2Convert() {}

Ros2Convert::~Ros2Convert() {}

void Ros2Convert::Init(std::shared_ptr<rclcpp::Node> nh,
                       std::shared_ptr<RuntimeConfig> param) {
  node   = nh;
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

void Ros2Convert::Run() {
  sleep(1);
  if (!param_->b_save_data) {
    std::cout << "b_save_data is false! " << std::endl;
    abort();
  }

  rosbag2_cpp::StorageOptions storage_options;
  std::string rosbag_read_path =
      param_->rosbag_path + "/" + param_->rosbag_name;
  std::cout << rosbag_read_path << std::endl;

  storage_options.uri = rosbag_read_path;
  // 默认 SQLite3
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format  = "cdr";
  converter_options.output_serialization_format = "cdr";
  // default
  // converter_options({rmw_get_serialization_format(),
  // rmw_get_serialization_format()});

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
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start local_pose.\033[0m");
  }
  if (param_->b_global_pose) {
    topics.push_back(param_->topic_global_pose_sub);
    RCLCPP_INFO(node->get_logger(),
                "\033[1;32m----> start global_pose.\033[0m");
  }
  if (param_->b_imu_data) {
    topics.push_back(param_->topic_imu_data_sub);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start imu_data.\033[0m");
  }

  // 第一次读取ros2bag，获取雷达的采样时间戳
  {
    auto reader_impl =
        std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    try {
      reader_impl->open(storage_options, converter_options);  // 打开ros2bag
    } catch (const std::runtime_error& e) {
      std::cerr << "Open Bag Wrong: " << e.what() << std::endl;
      return;
    }

    rosbag2_cpp::Reader reader(std::move(reader_impl));  // 读取ros2bag

    if (param_->b_lidar) {
      RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start lidar.\033[0m");

      if (param_->b_difop == 0) {
        while (reader.has_next()) {
          auto bag_msg = reader.read_next();
          auto& topic  = bag_msg->topic_name;
          if (topic == param_->topic_lidar_sub) {
            auto msg = DeserializeMsg<sensor_msgs::msg::PointCloud2>(bag_msg);
            this->LidarHandler(msg);
          }
          if (data_processor->b_final) {
            break;
          }
        }
      } else {
        while (reader.has_next()) {
          auto bag_msg = reader.read_next();
          auto& topic  = bag_msg->topic_name;

          // 没有设计多线程解锁，需要重新单线程
          this->SendLidarHandler(bag_msg, topic);
          if (data_processor->b_final) {
            break;
          }
        }
      }
    }
  }

  topics.clear();
  if (param_->b_camera) {
    for (size_t i = 0; i < param_->b_camera; ++i) {
      topics.push_back(param_->topic_camera_sub[i]);
      RCLCPP_INFO(node->get_logger(),
                  "\033[1;32m----> start camera_%zu.\033[0m", i + 1);
    }
  }
  if (param_->b_infra) {
    for (size_t i = 0; i < param_->b_infra; ++i) {
      topics.push_back(param_->topic_infra_sub[i]);
      RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start infra_%zu.\033[0m",
                  i + 1);
    }
  }
  if (param_->b_star) {
    for (size_t i = 0; i < param_->b_star; ++i) {
      topics.push_back(param_->topic_star_sub[i]);
      RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start star_%zu.\033[0m",
                  i + 1);
    }
  }

  // 第二次读取ros2bag，解析其他的数据
  {
    auto reader_impl =
        std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    try {
      reader_impl->open(storage_options, converter_options);  // 打开ros2bag
    } catch (const std::runtime_error& e) {
      std::cerr << "Open Bag Wrong: " << e.what() << std::endl;
      return;
    }

    rosbag2_cpp::Reader reader(std::move(reader_impl));  // 读取ros2bag

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      auto& topic  = bag_msg->topic_name;

      Ros2bagParseBase(bag_msg);

      if (param_->b_camera) {
        Ros2bagParseImageWrapper(bag_msg, topic, camera_topic_map, ds_camera);
      }

      if (param_->b_infra) {
        Ros2bagParseImageWrapper(bag_msg, topic, infra_topic_map, ds_infra);
      }

      if (param_->b_star) {
        Ros2bagParseImageWrapper(bag_msg, topic, star_topic_map, ds_star);
      }

      // 数据量较小，所以一块处理。
      if (param_->b_radar) {
        RadarHandler(bag_msg);
      }

      if (param_->b_radar4d) {
        auto it = radar4d_topic_map.find(topic);
        if (it != radar4d_topic_map.end()) {
          Radar4DHandler(bag_msg, it->second);
        }
      }
    }
  }

  data_processor->Stop();

  std::cout << "--- --- stop && end --- --- " << std::endl;
  sleep(2);

  // clang-format off
  RCLCPP_INFO(node->get_logger(), "----> message global num %d", num_global_pose);
  RCLCPP_INFO(node->get_logger(), "----> message local num %d", num_local_pose);
  RCLCPP_INFO(node->get_logger(), "----> message imu num %d", num_imu_data);
  RCLCPP_INFO(node->get_logger(), "----> message lidar send num %d", num_lidar_send);
  RCLCPP_INFO(node->get_logger(), "----> message lidar recv num %d", num_lidar_recv);
  PrintParserCount(ds_camera, "camera");
  PrintParserCount(ds_infra, "infra");
  PrintParserCount(ds_star, "star");
  RCLCPP_INFO(node->get_logger(), "----> message radar num %d", ds_radar.num);
  PrintParserCount(ds_radar4d, "radar4d");

  RCLCPP_INFO(node->get_logger(), "\033[1;32m----> txt file generated over.\033[0m");
  // clang-format on
  std::cout.flush();
  RCLCPP_INFO(node->get_logger(), "...");  // ROS日志自带刷新
  exit(1);
}

void Ros2Convert::Ros2bagParseBase(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) {
  const std::string& topic = m->topic_name;
  if (param_->b_local_pose && topic == param_->topic_local_pose_sub) {
    auto msg = DeserializeMsg<self_state::msg::LocalPose>(m);
    LocalPoseHandler(msg);
  } else if (param_->b_global_pose && topic == param_->topic_global_pose_sub) {
    auto msg = DeserializeMsg<self_state::msg::GlobalPose>(m);
    GlobalPoseHandler(msg);
  } else if (param_->b_imu_data && topic == param_->topic_imu_data_sub) {
    auto msg = DeserializeMsg<sensor_msgs::msg::Imu>(m);
    ImuDataHandler(msg);
  }
  // 可以继续根据其他话题做相应处理
}

void Ros2Convert::LocalPoseHandler(
    self_state::msg::LocalPose::ConstPtr msg_ptr) {
  if (msg_ptr != NULL) {
    auto& fp_local_pose = data_processor->fp_local_pose;

    // UGV2025 ROS_MSG
    // clang-format off
    // /*
    fprintf(fp_local_pose, "%lf %u "
            "%d %d %d %d %d %d  "
            "%d %d %d %d "
            "%d %d %d "
            "%d %d %d "
            "%d %d %d %d "
            "%d %d \n",
            msg_ptr->local_time,                      (uint32_t)msg_ptr->utc_time,
            (int)F_ROUND(msg_ptr->dr_x*100),          (int)F_ROUND(msg_ptr->dr_y*100),       (int)F_ROUND(msg_ptr->dr_z*100), 
            (int)F_ROUND(msg_ptr->dr_roll*100),       (int)F_ROUND(msg_ptr->dr_pitch*100),   (int)F_ROUND(msg_ptr->dr_heading*100),

            (int)F_ROUND(msg_ptr->speed_x*100),       (int)F_ROUND(msg_ptr->speed_y*100),    (int)F_ROUND(msg_ptr->speed_z*100), 
            (int)F_ROUND(msg_ptr->vehicle_speed*100),

            0, 0, 0,
            0, 0, 0,
            0, 0, 0, msg_ptr->driving_direction,
            0, 0);
    // */
    // clang-format on

    num_local_pose++;
  } else {
    RCLCPP_WARN(node->get_logger(), "the null local_pose message ...");
  }
}

void Ros2Convert::GlobalPoseHandler(
    self_state::msg::GlobalPose::ConstPtr msg_ptr) {
  if (msg_ptr != NULL) {
    auto& fp_global_pose = data_processor->fp_global_pose;

    // UGV2025 ROS_MSG
    // clang-format off
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
            msg_ptr->local_time,                      (uint32_t)msg_ptr->utc_time,
            msg_ptr->latitude*1e6, /*纬度*/            msg_ptr->longitude*1e6, /*经度*/
            (int)F_ROUND(msg_ptr->gauss_x*100),       (int)F_ROUND(msg_ptr->gauss_y*100),       (int)F_ROUND(msg_ptr->height*100),
            (int)F_ROUND(msg_ptr->roll*100),          (int)F_ROUND(msg_ptr->pitch*100),         (int)F_ROUND(msg_ptr->azimuth*100),
            (int)F_ROUND(msg_ptr->v_east*100),        (int)F_ROUND(msg_ptr->v_north*100),       (int)F_ROUND(msg_ptr->v_up*100),
            0, 0, 0,
            0, 0, 0,
            msg_ptr->dev_gauss_x,   msg_ptr->dev_gauss_y,   msg_ptr->dev_height,
            msg_ptr->dev_roll,      msg_ptr->dev_pitch,     msg_ptr->dev_azimuth,
            msg_ptr->dev_v_east,    msg_ptr->dev_v_north,   msg_ptr->dev_v_up,        

            msg_ptr->ins_status,    msg_ptr->pos_type,
            0, 0, 0, 0,
            0.0, 0.0, 0.0, 0.0);
    // */
    // clang-format on

    num_global_pose++;
  } else {
    RCLCPP_WARN(node->get_logger(), "the null global_pose message ...");
  }
}

void Ros2Convert::ImuDataHandler(
    const sensor_msgs::msg::Imu::ConstPtr msg_ptr) {
  if (msg_ptr != NULL) {
    auto& fp_imu_data = data_processor->fp_imu_data;

    double timestamp = toMs(msg_ptr->header.stamp);

    // CodexOpen ROS_MSG
    // clang-format off
    // cm | 0.01degree | cm/s |  cm/s^2 | 0.01degree/s | 0.01degree/s^2
    /*
    fprintf(fp_imu_data, "%lf %lf %d "
            "%d %d %d %d %d %d\n",
            timestamp,                                        0.0,                                              0,
            (int)F_ROUND(msg_ptr->angular_velocity.x*100),    (int)F_ROUND(msg_ptr->angular_velocity.y*100),    (int)F_ROUND(msg_ptr->angular_velocity.z*100),
            (int)F_ROUND(msg_ptr->linear_acceleration.x*100), (int)F_ROUND(msg_ptr->linear_acceleration.y*100), (int)F_ROUND(msg_ptr->linear_acceleration.z*100)
          );
    */
    // /*
    fprintf(fp_imu_data, "%lf %u %d "
            "%lf %lf %lf %lf %lf %lf\n",
            timestamp,                      0,                              0,
            msg_ptr->angular_velocity.x,    msg_ptr->angular_velocity.y,    msg_ptr->angular_velocity.z,
            msg_ptr->linear_acceleration.x, msg_ptr->linear_acceleration.y, msg_ptr->linear_acceleration.z
          );
    // */
    // clang-format on

    num_imu_data++;
  } else {
    RCLCPP_WARN(node->get_logger(), "the null imu_data message ...");
  }
}

void Ros2Convert::Ros2bagParseImageWrapper(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& m,
    const std::string& topic,
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
    task.compressed_msg = DeserializeMsg<sensor_msgs::msg::CompressedImage>(m);
    if (!task.compressed_msg) return;
  } else {
    task.type = ImageTask::ImageType::RAW;
    // 反序列化成 Image
    task.raw_msg = DeserializeMsg<sensor_msgs::msg::Image>(m);
    if (!task.raw_msg) return;
  }

  ImageWorkerFunc(task, data_processor.get(), param_.get());
}

void Ros2Convert::ImageWorkerFunc(const ImageTask& task, DataProcessor* proc,
                                  RuntimeConfig* param) {
  DataStatistic<cv::Mat>& ds = *task.ds;

  cv_bridge::CvImagePtr cv_ptr;

  try {
    if (task.type == ImageTask::ImageType::RAW) {
      const auto& msg = task.raw_msg;
      // RCLCPP_INFO(node->get_logger(), "encoding: %s", msg->encoding.c_str());

      if (msg->encoding == sensor_msgs::image_encodings::BGR8 ||
          msg->encoding == sensor_msgs::image_encodings::RGB8 ||
          msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv_ptr = cv_bridge::toCvCopy(task.raw_msg,
                                     sensor_msgs::image_encodings::BGR8);
      } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        // 深度图：不能转 BGR8
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      } else {
        RCLCPP_WARN(node->get_logger(), "Unsupported encoding: %s",
                    msg->encoding.c_str());
        return;
      }
    } else {
      cv_ptr = cv_bridge::toCvCopy(task.compressed_msg,
                                   sensor_msgs::image_encodings::BGR8);
    }
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_INFO(node->get_logger(), "cv_bridge exception in worker: %s",
                e.what());
    return;
  }

  // 获取时间戳
  // uint64_t msg_time =
  //     (task.type == ImageTask::ImageType::RAW)
  //         ? (task.raw_msg->header.stamp.sec * 1000ull +
  //            task.raw_msg->header.stamp.nanosec / 1000000ull)
  //         : (task.compressed_msg->header.stamp.sec * 1000ull +
  //            task.compressed_msg->header.stamp.nanosec / 1000000ull);
  uint64_t msg_time = (task.type == ImageTask::ImageType::RAW)
                          ? toMs(task.raw_msg->header.stamp)
                          : toMs(task.compressed_msg->header.stamp);

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

void Ros2Convert::RadarHandler(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) {
  auto type = SensorRegistry::Instance().GetRadarType(param_->radar_type);
  auto& path_radar = data_processor->path_radar;

  // /* ESR_Radar
  if (type == RadarType::ESR) {
    // UGV2025 ROS_MSG
    auto msg_ptr = DeserializeMsg<sensor::msg::EsrRadarInfo>(m);

    if (msg_ptr != NULL) {
      uint64_t msg_time = toMs(msg_ptr->header.stamp);

      RadarPoint Point;
      std::vector<RadarPoint> PointCloud;

      for (int i = 0; i < msg_ptr->object_num; i++) {
        Point.id = msg_ptr->object_data[i].target_i_d;
        Point.x  = msg_ptr->object_data[i].front_distance;
        Point.y  = msg_ptr->object_data[i].left_distance;

        Point.range      = msg_ptr->object_data[i].range;
        Point.azimuth    = msg_ptr->object_data[i].angle;
        Point.range_rate = msg_ptr->object_data[i].range_rate;
        Point.velocity   = msg_ptr->object_data[i].speed;
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
  }
  // */
  else {
    std::cout << "radar_type set error !!! " << std::endl;
  }
  ds_radar.num++;
}

void Ros2Convert::Radar4DHandler(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m, int idx) {
  // 4D毫米波雷达带速度和速度残差
  auto type = SensorRegistry::Instance().GetRadar4dType(param_->radar4d_type);
  auto& path_radar4d = data_processor->path_radar4d[idx];

  auto& ds = ds_radar4d[idx];

  // /* ars_548
  if (type == Radar4dType::ARS548) {
    auto msg_ptr = DeserializeMsg<ars548_interface::msg::DetectionList>(m);

    if (msg_ptr != NULL) {
      const auto& detections = msg_ptr->detections;

      // std::cout<<"start radar 4d "<<std::endl;
      uint64_t msg_time = toMs(detections[0].header.stamp);

      Radar4DPoint Point;
      std::vector<Radar4DPoint> PointCloud;
      // clang-format off
      // m ==> cm
      for (int i = 0; i < detections.size(); i++) {
        Point.x = detections[i].f_x * 100.0;
        Point.y = detections[i].f_y * 100.0;
        Point.z = detections[i].f_z * 100.0;
        Point.range_rate = detections[i].f_range_rate * 100.0;
        Point.range_rate_rms = detections[i].f_range_rate_std * 100.0;
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
  }
  // */
  // /* hugin_arbe
  else if (type == Radar4dType::ARBE) {
    auto msg_ptr = DeserializeMsg<sensor_msgs::msg::PointCloud2>(m);

    if (msg_ptr != NULL) {
      uint64_t msg_time = toMs(msg_ptr->header.stamp);

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

void Ros2Convert::LidarHandler(
    sensor_msgs::msg::PointCloud2::ConstPtr msg_ptr) {
  if (msg_ptr != NULL) {
    uint64_t msg_time = toMs(msg_ptr->header.stamp);

    if (!data_processor->PushSampledTime(msg_time)) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg_ptr, *Cloud);

    // warning
    pcl::transformPointCloud(*Cloud, *Cloud,
                             GetTransMatrix(param_->b_lt_none_rt));

    if (param_->b_save_data) {
      data_processor->SaveLidarData(Cloud, msg_time);
    }

    num_lidar_recv++;
  }
}

void Ros2Convert::InitRslidar() {
  sem_init(&sem_a, 0, 1);
  // sem_init(&sem_b, 0, 1);

  sub_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      param_->topic_lidar_sub, rclcpp::QoS(10),
      std::bind(&Ros2Convert::RecvLidarHandler, this, std::placeholders::_1));

  pub_difop = node->create_publisher<rslidar_msg::msg::RslidarPacket>(
      param_->topic_lidar_difop_sub /*rslidar_packets_difop*/, 10);

  b_first_pub_difop = true;
}

void Ros2Convert::SendLidarHandler(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m,
    const std::string& topic) {
  // std::cout << "topic: " << topic << std::endl;

  auto type     = SensorRegistry::Instance().GetLidarType(param_->lidar_type);
  int difop_num = SensorRegistry::Instance().GetDifopNum(type);

  /*rslidar_packets_difop*/
  if (topic == param_->topic_lidar_difop_sub) {
    if (b_first_pub_difop) {
      b_first_pub_difop = false;
      std::cout << "pub difop first.\n";
    }

    auto msg = DeserializeMsg<rslidar_msg::msg::RslidarPacket>(m);
    pub_difop->publish(*msg);
    num_lidar_send++;

    // 等待回调处理完成
    if (num_lidar_send % (difop_num) == 0) {
      sem_wait(&sem_a);
    }
  }
}

void Ros2Convert::RecvLidarHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  if (!msg_ptr) return;

  uint64_t msg_time = toMs(msg_ptr->header.stamp);
  // std::cout << "lidar msg_time: " << msg_time << std::endl;

  sem_post(&sem_a);

  if (!data_processor->PushSampledTime(msg_time)) return;

  // clang-format off
  // ros2 for intensity float
  pcl::PointCloud<robosense_ros::PointIF>::Ptr rs_cloud(new pcl::PointCloud<robosense_ros::PointIF>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // clang-format on

  pcl::fromROSMsg(*msg_ptr, *rs_cloud);
  RsToPcl(rs_cloud, Cloud);

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
