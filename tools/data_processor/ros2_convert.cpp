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
  if (param_->b_lidar && param_->b_difop) {
    this->InitRs();
  }

  ds_camera.resize(param_->b_camera, DataStatistic(1));
  ds_infra.resize(param_->b_infra, DataStatistic(2));
  ds_star.resize(param_->b_star, DataStatistic(3));
  ds_radar_4d.resize(param_->b_radar_4d);
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

  storage_options.uri        = rosbag_read_path;
  storage_options.storage_id = "sqlite3";  // 默认 SQLite3

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format  = "cdr";
  converter_options.output_serialization_format = "cdr";
  // default
  // converter_options({rmw_get_serialization_format(),
  // rmw_get_serialization_format()});

  data_processor->Start();

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

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();

      if (param_->b_lidar) {
        static bool first_run = true;
        if (first_run) {
          if (param_->b_difop == 0) {
            RCLCPP_INFO(node->get_logger(),
                        "\033[1;32m----> start lidar.\033[0m");
          } else {
            RCLCPP_INFO(node->get_logger(),
                        "\033[1;32m----> start lidar packets.\033[0m");
          }
          first_run = false;
        }

        if (data_processor->b_final) {
          break;
        }

        if (param_->b_difop == 0) {
          if (bag_msg->topic_name == param_->topic_lidar_sub) {
            auto msg = DeserializeMsg<sensor_msgs::msg::PointCloud2>(bag_msg);
            this->LidarHandler(msg);
          }
        } else {
          // 没有设计多线程解锁，需要重新单线程
          this->SendLidarHandler(bag_msg);
        }
      }
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

      if (param_->b_global_pose || param_->b_local_pose) {
        this->Ros2bagParseBase(bag_msg);
      }

      // clang-format off
      if(param_->b_camera){
        this->Ros2bagParseImageWrapper(
          bag_msg, param_->b_camera, param_->topic_camera_sub, ds_camera, "camera");
      }

      if(param_->b_infra){
        this->Ros2bagParseImageWrapper(
            bag_msg, param_->b_infra, param_->topic_infra_sub, ds_infra, "infra");
      }

      if(param_->b_star){
        this->Ros2bagParseImageWrapper(
            bag_msg, param_->b_star, param_->topic_star_sub, ds_star, "star");
      }
      // clang-format on

      // 数据量较小，所以一块处理。
      if (param_->b_radar || param_->b_radar_4d) {
        this->Ros2bagParseRadar(bag_msg);
      }
    }
  }

  data_processor->Stop();

  std::cout << "--- --- stop && end --- --- " << std::endl;

  sleep(2);
  // clang-format off
  RCLCPP_INFO(node->get_logger(), "----> message global num %d", num_global_pose);
  RCLCPP_INFO(node->get_logger(), "----> message local num %d", num_local_pose);

  RCLCPP_INFO(node->get_logger(), "----> message lidar send num %d", num_lidar_send);
  RCLCPP_INFO(node->get_logger(), "----> message lidar recv num %d", num_lidar_recv);

  PrintParserCount(ds_camera, "camera");
  PrintParserCount(ds_infra, "infra");
  PrintParserCount(ds_star, "star");

  RCLCPP_INFO(node->get_logger(), "----> message radar num %d", ds_radar.num);

  PrintParserCount(ds_radar_4d, "radar_4d");

  RCLCPP_INFO(node->get_logger(), "\033[1;32m----> txt file generated over.\033[0m");
  // clang-format on

  std::cout.flush();
  RCLCPP_INFO(node->get_logger(), "...");  // ROS日志自带刷新
  exit(1);
}

void Ros2Convert::Ros2bagParseBase(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) {
  std::string topic = m->topic_name;
  if (topic == param_->topic_local_pose_sub) {
    if (num_local_pose == 0) {
      RCLCPP_INFO(node->get_logger(),
                  "\033[1;32m----> start local_pose.\033[0m");
    }
    auto msg = DeserializeMsg<self_state::msg::LocalPose>(m);
    LocalPoseHandler(msg);
    num_local_pose++;
  } else if (topic == param_->topic_global_pose_sub) {
    if (num_global_pose == 0) {
      RCLCPP_INFO(node->get_logger(),
                  "\033[1;32m----> start global_pose.\033[0m");
    }
    auto msg = DeserializeMsg<self_state::msg::GlobalPose>(m);
    GlobalPoseHandler(msg);
    num_global_pose++;
  } else if (topic == param_->topic_imu_data_sub) {
    if (num_imu_data == 0) {
      RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start imu_data.\033[0m");
    }
    // auto msg = DeserializeMsg<self_state::msg::Imu>(m);
    // ImuDataHandler(msg);
    num_imu_data++;
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
    fprintf(fp_local_pose, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %lf %lf %lf %lf\n",
            msg_ptr->local_time,          msg_ptr->utc_time,            msg_ptr->message_num,
            msg_ptr->dr_x,                msg_ptr->dr_y,                msg_ptr->dr_z,                
            msg_ptr->dr_heading,          msg_ptr->dr_roll,             msg_ptr->dr_pitch, 
            msg_ptr->vehicle_speed, 
            msg_ptr->speed_x,             msg_ptr->speed_y,             msg_ptr->speed_z, 
            0, 1, -1,
            msg_ptr->driving_direction,
            msg_ptr->reserved[0],         msg_ptr->reserved[1],
            msg_ptr->reserved[2],         msg_ptr->reserved[3]);
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
    fprintf(fp_global_pose, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %lf %lf\n",
            msg_ptr->local_time,      msg_ptr->utc_time,        msg_ptr->message_num,
            msg_ptr->gauss_x,         msg_ptr->gauss_y,         msg_ptr->height,          
            msg_ptr->v_east,           msg_ptr->v_north,        msg_ptr->v_up,            
            msg_ptr->azimuth,          msg_ptr->pitch,          msg_ptr->roll,

            msg_ptr->dev_gauss_x,     msg_ptr->dev_gauss_y,
            msg_ptr->dev_height,      msg_ptr->dev_v_east,      msg_ptr->dev_v_north,
            msg_ptr->dev_v_up,        msg_ptr->dev_azimuth,     msg_ptr->dev_pitch,
            msg_ptr->dev_roll,

            msg_ptr->longitude,       msg_ptr->latitude,

            msg_ptr->ins_status,      msg_ptr->pos_type,
            msg_ptr->reserved[0],     msg_ptr->reserved[1],
            msg_ptr->reserved[2],     msg_ptr->reserved[3]);
    // */
    // clang-format on

    num_global_pose++;
  } else {
    RCLCPP_WARN(node->get_logger(), "the null global_pose message ...");
  }
}

void Ros2Convert::Ros2bagParseImageWrapper(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& m,
    int sensor_count, const std::vector<std::string>& topics,
    std::vector<DataStatistic>& ds, const std::string& parser_name) {
  for (int i = 0; i < sensor_count; i++) {
    if (ds.at(i).num == 0) {
      RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start %s_%d.\033[0m",
                  parser_name.c_str(), i + 1);
    }

    if (param_->prepare_data_num != -1 &&
        data_processor->IsEnd(ds.at(i).sampled_index)) {
      break;
    }

    // clang-format off
    if (m->topic_name == topics.at(i)) {
      if (param_->b_compressed) {
        // 反序列化成 CompressedImage
        auto msg = DeserializeMsg<sensor_msgs::msg::CompressedImage>(m);
        CameraCompressedHandler(msg, i, ds.at(i).mode, ds.at(i).sampled_index, ds.at(i).num);
      } else {
        // 反序列化成 Image
        auto msg = DeserializeMsg<sensor_msgs::msg::Image>(m);
        CameraHandler(msg, i, ds.at(i).mode, ds.at(i).sampled_index, ds.at(i).num);
      }
      // clang-format on
    }
  }
}

void Ros2Convert::CameraHandler(sensor_msgs::msg::Image::ConstPtr msg_ptr,
                                int id, int mode, size_t& sampled_index,
                                int& num_image) {
  if (!msg_ptr) return;

  uint64_t msg_time = msg_ptr->header.stamp.sec * 1000ull +
                      msg_ptr->header.stamp.nanosec / 1000000ull;
  // std::cout << "image msg_time: " << msg_time << std::endl;

  // check the time range
  if (!data_processor->CheckSampledTime(msg_time, sampled_index)) return;

  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
  // cv::Mat tmp_image = cv_ptr->image.clone();  // 深拷贝，防止数据丢失
  cv::Mat tmp_image = cv_ptr->image;  // 浅拷贝

  data_processor->ProcessCameraImage(tmp_image, msg_time, id, mode);
  num_image++;
}

void Ros2Convert::CameraCompressedHandler(
    sensor_msgs::msg::CompressedImage::ConstPtr msg_ptr, int id, int mode,
    size_t& sampled_index, int& num_image) {
  if (!msg_ptr) return;

  uint64_t msg_time = msg_ptr->header.stamp.sec * 1000ull +
                      msg_ptr->header.stamp.nanosec / 1000000ull;
  // std::cout << "image msg_time: " << msg_time << std::endl;

  // check the time range
  if (!data_processor->CheckSampledTime(msg_time, sampled_index)) return;

  cv_bridge::CvImagePtr cv_ptr_compressed =
      cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
  cv::Mat tmp_image = cv_ptr_compressed->image;

  // std::cout << "id : " << id << std::endl;
  data_processor->ProcessCameraImage(tmp_image, msg_time, id, mode);
  num_image++;
}

void Ros2Convert::Ros2bagParseRadar(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) {
  std::string topic = m->topic_name;
  if (topic == param_->topic_radar_sub) {
    if (ds_radar.num == 0) {
      RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start radar.\033[0m");
    }
    RadarHandler(m);
    ds_radar.num++;
  } else {
    // clang-format off
    this->Ros2bagParseRadar4DWrapper(
        m, param_->b_radar_4d, param_->topic_radar_4d_sub, ds_radar_4d, "radar_4d");
    // clang-format on
  }
}

void Ros2Convert::RadarHandler(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) {
  static int type  = RadarTypeParam[param_->radar_type];
  auto& path_radar = data_processor->path_radar;

  // /* ESR_Radar
  if (type == 1) {
    // UGV2025 ROS_MSG
    auto msg_ptr = DeserializeMsg<sensor::msg::EsrRadarInfo>(m);

    if (msg_ptr != NULL) {
      uint64_t msg_time = msg_ptr->header.stamp.sec * 1000ull +
                          msg_ptr->header.stamp.nanosec / 1000000ull;

      RadarPoint Point;
      std::vector<RadarPoint> PointCloud;

      for (int i = 0; i < msg_ptr->object_num; i++) {
        Point.id         = msg_ptr->object_data[i].target_i_d;
        Point.x          = msg_ptr->object_data[i].front_distance;
        Point.y          = msg_ptr->object_data[i].left_distance;
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

        int tmp_id;
        float tmp_x, tmp_y;
        float tmp_range, tmp_angle;
        float tmp_range_rate, tmp_v;
        for (int i = 0; i < PointCloud.size(); i++) {
          tmp_id         = PointCloud[i].id;
          tmp_x          = float(PointCloud[i].x);
          tmp_y          = float(PointCloud[i].y);
          tmp_range      = PointCloud[i].range;
          tmp_angle      = float(PointCloud[i].azimuth);
          tmp_range_rate = float(PointCloud[i].range_rate);
          tmp_v          = float(PointCloud[i].velocity);

          // clang-format off
          fprintf(fp_radar, "%d %f %f %f %f %f %f\n",
                  tmp_id, tmp_x, tmp_y, tmp_range, tmp_angle),
                  tmp_range_rate, tmp_v;
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
}

void Ros2Convert::Ros2bagParseRadar4DWrapper(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& m,
    int sensor_count, const std::vector<std::string>& topics,
    std::vector<DataStatistic>& ds, const std::string& parser_name) {
  for (int i = 0; i < sensor_count; i++) {
    if (ds.at(i).num == 0) {
      RCLCPP_INFO(node->get_logger(), "\033[1;32m----> start %s_%d.\033[0m",
                  parser_name.c_str(), i + 1);
    }

    if (param_->prepare_data_num != -1 &&
        data_processor->IsEnd(ds.at(i).sampled_index)) {
      break;
    }

    if (m->topic_name == topics.at(i)) {
      Radar4DHandler(m, i, ds.at(i).sampled_index, ds.at(i).num);
    }
  }
}

void Ros2Convert::Radar4DHandler(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m, int id,
    size_t& sampled_index, int& num_radar) {
  static int type     = Radar4DTypeParam[param_->radar_4d_type];
  auto& path_radar_4d = data_processor->path_radar_4d.at(id);

  // /* ars_548
  if (type == 1) {
    auto msg_ptr = DeserializeMsg<ars548_interface::msg::DetectionList>(m);

    if (msg_ptr != NULL) {
      // std::cout<<"start radar 4d "<<std::endl;
      uint64_t msg_time =
          msg_ptr->detections[0].header.stamp.sec * 1000ull +
          msg_ptr->detections[0].header.stamp.nanosec / 1000000ull;

      Radar4DPoint Point;
      std::vector<Radar4DPoint> PointCloud;
      for (int i = 0; i < msg_ptr->detections.size(); i++) {
        Point.x              = msg_ptr->detections[i].f_x;
        Point.y              = msg_ptr->detections[i].f_y;
        Point.z              = msg_ptr->detections[i].f_z;
        Point.range_rate     = msg_ptr->detections[i].f_range_rate;
        Point.range_rate_rms = msg_ptr->detections[i].f_range_rate_std;
        // Add other fields as necessary.
        PointCloud.push_back(Point);
      }

      char file_radar_4d[300];
      if (param_->use_txt_or_pcd == 0) {
        sprintf(file_radar_4d, "%s/%ld.txt", path_radar_4d.c_str(), msg_time);
        FILE* fp_radar_4d;
        fp_radar_4d = fopen(file_radar_4d, "w");
        if (fp_radar_4d == NULL) {
          perror("Radar4D file create error!");
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
          fprintf(fp_radar_4d, "%f %f %f %f %f\n", 
                  tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv);
          // clang-format on
        }
        fclose(fp_radar_4d);
      }
      PointCloud.clear();
    }
  }
  // */
  // /* hugin_arbe
  else if (type == 2) {
    auto msg_ptr = DeserializeMsg<sensor_msgs::msg::PointCloud2>(m);

    if (msg_ptr != NULL) {
      uint64_t msg_time = msg_ptr->header.stamp.sec * 1000ull +
                          msg_ptr->header.stamp.nanosec / 1000000ull;

      hugin::PointCloud PointCloud;

      pcl::fromROSMsg(*msg_ptr, PointCloud.data);
      PointCloud.timestamp = msg_time;

      char file_radar_4d[300];
      if (param_->use_txt_or_pcd == 0) {
        sprintf(file_radar_4d, "%s/%ld.txt", path_radar_4d.c_str(), msg_time);
        FILE* fp_radar_4d;
        fp_radar_4d = fopen(file_radar_4d, "w");
        if (fp_radar_4d == NULL) {
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
          fprintf(fp_radar_4d, "%f %f %f %f %f %f %f %f\n", 
                  tmp_x, tmp_y, tmp_z, tmp_range, 
                  tmp_azimuth, tmp_elevation, tmp_doppler,
                  tmp_rcs);
          // clang-format on
          fclose(fp_radar_4d);
        }
      }
    }
  }
  // */
  else {
    std::cout << "radar_4d_type set error !!! " << std::endl;
  }
  ds_radar_4d.at(id).num++;
  // std::cout << ds_radar_4d.at(id) << std::endl;
}

void Ros2Convert::LidarHandler(
    sensor_msgs::msg::PointCloud2::ConstPtr msg_ptr) {
  if (msg_ptr != NULL) {
    uint64_t msg_time = msg_ptr->header.stamp.sec * 1000ull +
                        msg_ptr->header.stamp.nanosec / 1000000ull;

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

void Ros2Convert::InitRs() {
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
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) {
  static int difop_num = LidarTypeParam[param_->lidar_type];

  std::string topic = m->topic_name;
  // std::cout << "topic: " << topic << std::endl;

  if (topic == param_->topic_lidar_difop_sub /*rslidar_packets_difop*/) {
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

  uint64_t msg_time = msg_ptr->header.stamp.sec * 1000ull +
                      msg_ptr->header.stamp.nanosec / 1000000ull;
  std::cout << "lidar msg_time: " << msg_time << std::endl;

  sem_post(&sem_a);

  if (!data_processor->PushSampledTime(msg_time)) return;

  // clang-form off
  // ros2 for intensity float
  pcl::PointCloud<robosense_ros::PointIF>::Ptr rs_cloud(
      new pcl::PointCloud<robosense_ros::PointIF>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  // clang-form on

  pcl::fromROSMsg(*msg_ptr, *rs_cloud);
  RsToPcl(rs_cloud, Cloud);

  // 去除 NaN 点
  // std::cout << "Before filter: " << Cloud->size() << " points" << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<int> indices;  // 存储有效点的索引
  pcl::removeNaNFromPointCloud(*Cloud, *FilteredCloud, indices);
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

void Ros2Convert::PrintParserCount(const std::vector<DataStatistic>& counts,
                                   const std::string& name) {
  int size = counts.size();

  if (size == 0) {
    RCLCPP_INFO(node->get_logger(), "----> message %s num %d", name.c_str(), 0);
  } else if (size == 1) {
    RCLCPP_INFO(node->get_logger(), "----> message %s num %d", name.c_str(),
                counts.at(0).num);
  } else {
    for (int i = 0; i < size; ++i) {
      RCLCPP_INFO(node->get_logger(), "----> message %s_%d num %d",
                  name.c_str(), i + 1, counts.at(i).num);
    }
  }
}

}  // namespace tools
}  // namespace jojo
