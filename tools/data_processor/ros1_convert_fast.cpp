#include "tools/data_processor/ros1_convert_fast.h"

#define foreach BOOST_FOREACH

/*  注意检查输入点云的 坐标系 是右前上 还是前左上
 *  并且检查是否在内部做了二次转换
 *  pcl::transformPointCloud(cloud, cloud, transform_mat);
 */

/*  ===================================================================
 *  多线程架构说明（重要）
 *  ===================================================================
 *
 *  rosbag::Bag / MessageInstance 线程不安全根因：
 *    - m.instantiate<T>() 内部调用 bag.readMessages()
 *    - 该函数修改 Bag 内部共享状态：file offset、decompress buffer、cache
 *    - 多线程并发调用必然导致数据损坏或崩溃
 *
 *  解决方案：Reader-Worker 分离架构
 *    [Reader 主线程]
 *      - 独占 rosbag::Bag 和 rosbag::View 迭代器
 *      - 在主线程完成所有 m.instantiate<T>()，把消息序列化为 ImageTask
 *      - 将 ImageTask push 进 per-channel BlockingQueue
 *
 *    [Worker 线程池（每个 channel 一个线程）]
 *      - 只做 CPU 密集型工作：cv_bridge::toCvCopy + ProcessCameraImage
 *      - 完全不接触 bag 对象
 *      - 通过 BlockingQueue 解耦，Reader 不会因 Worker 慢而阻塞
 *
 *  关键约束：
 *    1. m.instantiate<T>() 必须且只能在 Reader 主线程调用
 *    2. bag.close() 必须等所有 Worker 消费完 queue 后再调用
 *    3. DataStatistic 的 sampled_index / diff 等字段由 Worker 独占访问，
 *       每个 channel 对应独立 Worker，因此无需加锁
 *    4. data_processor->ProcessCameraImage 如有内部共享状态需自行加锁
 *  ===================================================================
 */

namespace jojo {
namespace tools {

/**
 * @brief 图像处理 Worker 线程入口
 *
 * 每个 channel（camera[0], camera[1], infra[0]…）对应一个独立 Worker，
 * 因此 DataStatistic（ds）由该 Worker 独占，无需加锁。
 *
 * @param queue   对应 channel 的任务队列
 * @param proc    DataProcessor 指针（需保证 ProcessCameraImage 线程安全）
 * @param param   配置参数
 */
static void ImageWorkerFunc(BlockingQueue<ImageTask>* queue,
                            DataProcessor* proc, RuntimeConfig* param) {
  ImageTask task;
  while (queue->Pop(task)) {
    DataStatistic<cv::Mat>& ds = *task.ds;

    cv_bridge::CvImagePtr cv_ptr;

    try {
      if (task.type == ImageTask::ImageType::RAW) {
        cv_ptr = cv_bridge::toCvCopy(task.raw_msg,
                                     sensor_msgs::image_encodings::BGR8);
      } else {
        cv_ptr = cv_bridge::toCvCopy(task.compressed_msg,
                                     sensor_msgs::image_encodings::BGR8);
      }
    } catch (const cv_bridge::Exception& e) {
      ROS_WARN("cv_bridge exception in worker: %s", e.what());
      continue;
    }

    // 获取时间戳
    uint64_t msg_time =
        (task.type == ImageTask::ImageType::RAW)
            ? (task.raw_msg->header.stamp.toSec() * 1000)
            : (task.compressed_msg->header.stamp.toSec() * 1000);

    // 检查采样时间窗口（CheckSampledTime 内部有锁，可并发调用）
    int64_t diff = 0;
    if (!proc->CheckSampledTime(msg_time, ds.sampled_index, diff)) continue;

    if (param->prepare_data_num != -1) {
      // 有采样数量限制：寻找最接近采样点的帧
      if (std::abs(diff) < std::abs(ds.diff)) {
        ds.diff     = diff;
        ds.msg_time = msg_time;
        ds.data     = cv_ptr->image;
        // 还不是最接近的，继续等下一帧
        continue;
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
  // queue 关闭且清空后，线程自然退出
}

Ros1Convert::Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_  = nh;
  pnh_ = private_nh;
}

Ros1Convert::~Ros1Convert() {
  if (lpose_writer_) {
    lpose_writer_->Stop();
    delete lpose_writer_;
  }
  if (gpose_writer_) {
    gpose_writer_->Stop();
    delete gpose_writer_;
  }
  if (imu_writer_) {
    imu_writer_->Stop();
    delete imu_writer_;
  }
}

bool Ros1Convert::Init(std::shared_ptr<jojo::tools::RuntimeConfig> rparam,
                       std::shared_ptr<jojo::tools::InterfaceConfig> iparam) {
  rparam_ = rparam;
  iparam_ = iparam;

  data_processor = std::make_shared<DataProcessor>();
  data_processor->Init(rparam_, iparam_);

  // 这个要提前很久初始化，可能和ROS启动有关
  if (iparam_->b_lidar) {
    if (iparam_->b_difop) {
      this->InitRslidar();
    }

#if defined(RSLIDAR_OLD)
    // ros1 for intensity int8
    cloud_buffer_.reset(new pcl::PointCloud<robosense_ros::PointII>);
#elif defined(RSLIDAR_NEW)
    cloud_buffer_.reset(new pcl::PointCloud<robosense_ros::PointIF>);
#elif defined(VELODYNE)
    cloud_buffer_.reset(new pcl::PointCloud<velodyne_ros::PointXYZIR>);
#elif defined(LIVOX_NEW)
    cloud_buffer_.reset(new pcl::PointCloud<livox_ros::PointXYZIRT>);
#else
    cloud_buffer_.reset(new pcl::PointCloud<robosense_ros::Point>);
#endif
    // 根据雷达点数预估
    cloud_buffer_->points.reserve(230400);
#if defined(LIDAR_DATASET)
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZIRT>);
#else
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
#endif
    cloud_->points.reserve(230400);
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
    std::cout << "b_save_data is false! " << std::endl;
    abort();
  }

  // ── 打开 bag（只在主线程操作）──────────────────────────────
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

  /* ---------------- topic map ---------------- */
  for (size_t i = 0; i < iparam_->topic_camera_sub.size(); i++)
    camera_topic_map[iparam_->topic_camera_sub[i]] = i;
  for (size_t i = 0; i < iparam_->topic_infra_sub.size(); i++)
    infra_topic_map[iparam_->topic_infra_sub[i]] = i;
  for (size_t i = 0; i < iparam_->topic_star_sub.size(); i++)
    star_topic_map[iparam_->topic_star_sub[i]] = i;
  for (size_t i = 0; i < iparam_->topic_radar4d_sub.size(); i++)
    radar4d_topic_map[iparam_->topic_radar4d_sub[i]] = i;

  /* ---------------- topic 收集 ---------------- */
  std::vector<std::string> topics;
  if (iparam_->b_local_pose) {
    lpose_writer_ = new AsyncWriter(data_processor->fp_local_pose);
    topics.push_back(iparam_->topic_local_pose_sub);
    ROS_INFO("\033[1;32m----> start local_pose.\033[0m");
  }
  if (iparam_->b_global_pose) {
    gpose_writer_ = new AsyncWriter(data_processor->fp_global_pose);
    topics.push_back(iparam_->topic_global_pose_sub);
    ROS_INFO("\033[1;32m----> start global_pose.\033[0m");
  }
  if (iparam_->b_imu_data) {
    imu_writer_ = new AsyncWriter(data_processor->fp_imu_data);
    topics.push_back(iparam_->topic_imu_data_sub);
    ROS_INFO("\033[1;32m----> start imu_data.\033[0m");
  }
  if (iparam_->b_lidar) {
    ROS_INFO("\033[1;32m----> start lidar.\033[0m");
    if (iparam_->b_difop == 0)
      topics.push_back(iparam_->topic_lidar_sub);
    else {
      topics.push_back(iparam_->topic_lidar_ori_sub);
      topics.push_back(iparam_->topic_lidar_difop_sub);
    }
  }
  if (iparam_->b_camera) {
    for (size_t i = 0; i < iparam_->b_camera; ++i) {
      topics.push_back(iparam_->topic_camera_sub[i]);
      ROS_INFO("\033[1;32m----> start camera_%zu.\033[0m", i + 1);
    }
  }
  if (iparam_->b_infra) {
    for (size_t i = 0; i < iparam_->b_infra; ++i) {
      topics.push_back(iparam_->topic_infra_sub[i]);
      ROS_INFO("\033[1;32m----> start infra_%zu.\033[0m", i + 1);
    }
  }
  if (iparam_->b_star) {
    for (size_t i = 0; i < iparam_->b_star; ++i) {
      topics.push_back(iparam_->topic_star_sub[i]);
      ROS_INFO("\033[1;32m----> start star_%zu.\033[0m", i + 1);
    }
  }
  if (iparam_->b_radar) {
    topics.push_back(iparam_->topic_radar_sub);
    ROS_INFO("\033[1;32m----> start radar.\033[0m");
  }
  if (iparam_->b_radar4d) {
    for (size_t i = 0; i < iparam_->b_radar4d; ++i) {
      topics.push_back(iparam_->topic_radar4d_sub[i]);
      ROS_INFO("\033[1;32m----> start radar4d_%zu.\033[0m", i + 1);
    }
  }

  this->CreateImageWorker();

  // ── rosbag View ──────────────────────────────────────────
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // ── Reader 主循环 ─────────────────────────────────────────
  //
  //  所有 m.instantiate<T>() 必须在这里（主线程）完成，
  //  不得将 MessageInstance 的引用或指针传递给任何其他线程！
  //
  for (const rosbag::MessageInstance& m : view) {
    const std::string& topic = m.getTopic();

    // ── 轻量数据（pose/imu）：主线程直接处理 ────────────────
    Ros1bagParseBase(m);

    // ── Lidar：必须单线程 ─────────────────────────────────
    if (iparam_->b_lidar) {
      auto type = SensorRegistry::Instance().GetLidarType(rparam_->lidar_type);

      switch (type) {
        case LidarType::M1P:
        case LidarType::RS128:
          if (iparam_->b_difop == 0) {
            if (topic == iparam_->topic_lidar_sub) {
              // instantiate 在主线程
              auto msg = m.instantiate<sensor_msgs::PointCloud2>();
              this->LidarHandler(msg);
            }
          } else {
            this->SendLidarHandler(m, topic);
          }
          break;

        case LidarType::MID360:
          if (topic == iparam_->topic_lidar_sub) {
            // std::cout << "MID360: " << iparam_->topic_lidar_sub << std::endl;
            // instantiate 在主线程
#if defined(LIVOX_OLD)
            auto msg = m.instantiate<livox_ros_driver2::CustomMsg>();
            this->LivoxLidarHandler(msg);
#elif defined(LIVOX_NEW)
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            this->LidarHandler(msg);
#endif
          }
          break;

        default:
          auto msg = m.instantiate<sensor_msgs::PointCloud2>();
          this->NormalLidarHandler(msg);
          break;
      }
    }

    // ── 采样结束判断 ──────────────────────────────────────
    if (rparam_->prepare_data_num != -1) {
      if (data_processor->b_final) break;
      continue;  // 有采样数量限制时，图像通过 LidarHandler 驱动，不走下方分发
    }

    // ── Radar（不涉及图像，主线程直接处理）────────────────
    if (iparam_->b_radar) {
      RadarHandler(m);
    }

    if (iparam_->b_radar4d) {
      auto it = radar4d_topic_map.find(topic);
      if (it != radar4d_topic_map.end()) {
        Radar4DHandler(m, it->second);
      }
    }

    // ── 图像分发：主线程 instantiate，Worker 线程解码 ─────
    //
    //  关键：m.instantiate<T>() 在主线程调用，返回 shared_ptr，
    //  生命周期与 bag 迭代器解耦，可安全跨线程传递。
    //
    // camera
    if (iparam_->b_camera) {
      TryDispatchImageTask(m, topic, camera_topic_map, camera_queues,
                           ds_camera);
    }

    // infra
    if (iparam_->b_infra) {
      TryDispatchImageTask(m, topic, infra_topic_map, infra_queues, ds_infra);
    }

    // star
    if (iparam_->b_star) {
      TryDispatchImageTask(m, topic, star_topic_map, star_queues, ds_star);
    }
  }  // end for m : view
  std::cout << "end for m : view" << std::endl;

  // ── 关闭所有队列，等待 Worker 处理完毕后退出 ──────────────
  //
  //  必须先 Close() 所有队列，再 join() Worker 线程，
  //  bag.close() 必须在 join() 之后，确保 Worker 不再访问任何
  //  来自 bag 的共享数据（虽然 shared_ptr 已经保证安全，但这样更清晰）。
  //
  for (auto& q : camera_queues) q->Close();
  for (auto& q : infra_queues) q->Close();
  for (auto& q : star_queues) q->Close();

  for (auto& t : camera_workers)
    if (t.joinable()) t.join();
  for (auto& t : infra_workers)
    if (t.joinable()) t.join();
  for (auto& t : star_workers)
    if (t.joinable()) t.join();

  // ── 所有 Worker 已退出，可以安全关闭 bag ─────────────────
  bag.close();

  data_processor->Stop();

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
  if (iparam_->b_local_pose && topic == iparam_->topic_local_pose_sub) {
    auto msg = m.instantiate<self_state::LocalPose>();
    LocalPoseHandler(msg);
  } else if (iparam_->b_global_pose &&
             topic == iparam_->topic_global_pose_sub) {
    auto msg = m.instantiate<self_state::GlobalPose>();
    GlobalPoseHandler(msg);
  } else if (iparam_->b_imu_data && topic == iparam_->topic_imu_data_sub) {
    auto msg = m.instantiate<sensor_msgs::Imu>();
    ImuDataHandler(msg);
  }
  // 可以继续根据其他话题做相应处理
}

void Ros1Convert::LocalPoseHandler(
    const self_state::LocalPose::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    char buf[512];

    // CodexOpen ROS_MSG
    // clang-format off
    // cm | 0.01degree | cm/s |  cm/s^2 | 0.01degree/s | 0.01degree/s^2
    int n = snprintf(buf, sizeof(buf), 
            "%lf %u "
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

    lpose_writer_->Push(buf, n);
    num_local_pose++;
  } else {
    ROS_WARN("the null local_pose message ...");
  }
}

void Ros1Convert::GlobalPoseHandler(
    const self_state::GlobalPose::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    char buf[512];

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
    int n = snprintf(buf, sizeof(buf),
            "%lf %u "
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

    gpose_writer_->Push(buf, n);
    num_global_pose++;
  } else {
    ROS_WARN("the null global_pose message ...");
  }
}

void Ros1Convert::ImuDataHandler(const sensor_msgs::Imu::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    char buf[512];

    double timestamp = msg_ptr->header.stamp.toSec() * 1000;

    auto type = SensorRegistry::Instance().GetImuType(rparam_->imu_type);

    // CodexOpen ROS_MSG
    // cm | 0.01degree | cm/s |  cm/s^2 | 0.01degree/s | 0.01degree/s^2
    /*
    fprintf(fp_imu_data, "%lf %lf %d "
            "%d %d %d %d %d %d\n",
            timestamp,                                        0.0,                                              msg_ptr->header.seq,
            (int)F_ROUND(msg_ptr->angular_velocity.x*100),    (int)F_ROUND(msg_ptr->angular_velocity.y*100),    (int)F_ROUND(msg_ptr->angular_velocity.z*100),
            (int)F_ROUND(msg_ptr->linear_acceleration.x*100), (int)F_ROUND(msg_ptr->linear_acceleration.y*100), (int)F_ROUND(msg_ptr->linear_acceleration.z*100)
          );
    */
    int n = -1;

    if (type == ImuType::MID360) {
      // /*
      // clang-format off
      n = snprintf(buf, sizeof(buf),
              "%lf %u %d "
              "%lf %lf %lf %lf %lf %lf\n",
              timestamp,                           0,                                   msg_ptr->header.seq,
              msg_ptr->angular_velocity.x,         msg_ptr->angular_velocity.y,         msg_ptr->angular_velocity.z,
              msg_ptr->linear_acceleration.x * 10, msg_ptr->linear_acceleration.y * 10, msg_ptr->linear_acceleration.z * 10
            );
      // clang-format on
      // */
    } else if (type == ImuType::UNKNOWN) {
      // /*
      // clang-format off
      // 角度转弧度 && 右前上 转 前左上
      double imu_avx =  msg_ptr->angular_velocity.y * DEG_2_RAD;
      double imu_avy = -msg_ptr->angular_velocity.x * DEG_2_RAD;
      double imu_avz =  msg_ptr->angular_velocity.z * DEG_2_RAD;

      n = snprintf(buf, sizeof(buf),
              "%lf %u %d "
              "%lf %lf %lf %lf %lf %lf\n",
              timestamp,                      0,                              msg_ptr->header.seq,
              imu_avx,                        imu_avy,                        imu_avz,
              msg_ptr->linear_acceleration.x, msg_ptr->linear_acceleration.y, msg_ptr->linear_acceleration.z
            );
      // clang-format on
      // */
    }

    imu_writer_->Push(buf, n);
    num_imu_data++;
  } else {
    ROS_WARN("the null imu_data message ...");
  }
}

void Ros1Convert::CreateImageWorker() {
  // ── 创建 per-channel BlockingQueue ───────────────────────
  //
  //  队列大小 = 32：Reader 最多比 Worker 超前 32 帧，
  //  防止内存无限增长（图像帧通常较大）。
  //  可根据实际内存调整。
  //
  const size_t QUEUE_DEPTH = 32;

  // camera channels
  camera_queues.resize(iparam_->b_camera);
  for (int i = 0; i < iparam_->b_camera; i++) {
    camera_queues[i] = std::make_unique<BlockingQueue<ImageTask>>(QUEUE_DEPTH);
    camera_workers.emplace_back(ImageWorkerFunc, camera_queues[i].get(),
                                data_processor.get(), rparam_.get());
  }

  // infra channels
  infra_queues.resize(iparam_->b_infra);
  for (int i = 0; i < iparam_->b_infra; i++) {
    infra_queues[i] = std::make_unique<BlockingQueue<ImageTask>>(QUEUE_DEPTH);
    infra_workers.emplace_back(ImageWorkerFunc, infra_queues[i].get(),
                               data_processor.get(), rparam_.get());
  }

  // star channels
  star_queues.resize(iparam_->b_star);
  for (int i = 0; i < iparam_->b_star; i++) {
    star_queues[i] = std::make_unique<BlockingQueue<ImageTask>>(QUEUE_DEPTH);
    star_workers.emplace_back(ImageWorkerFunc, star_queues[i].get(),
                              data_processor.get(), rparam_.get());
  }
}

void Ros1Convert::TryDispatchImageTask(
    const rosbag::MessageInstance& m, const std::string& topic,
    const std::unordered_map<std::string, int>& topic_map,
    std::vector<std::unique_ptr<BlockingQueue<ImageTask>>>& queues,
    std::vector<DataStatistic<cv::Mat>>& ds_vec) {
  auto it = topic_map.find(topic);
  if (it == topic_map.end()) return;

  size_t idx = it->second;

  ImageTask task;
  task.channel_idx = static_cast<int>(idx);
  task.ds          = &ds_vec[idx];
  task.sensor_mode = ds_vec[idx].mode;

  if (iparam_->b_compressed) {
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

  queues[idx]->Push(std::move(task));
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
        Point.x = msg_ptr->objectData[i].x;
        Point.y = msg_ptr->objectData[i].y;
        // version 1.1
        // Point.x          = msg_ptr->objectData[i].front_distance;
        // Point.y          = msg_ptr->objectData[i].left_distance;
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
      if (rparam_->use_txt_or_pcd == 0) {
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
  auto type = SensorRegistry::Instance().GetRadar4dType(rparam_->radar4d_type);
  auto& path_radar4d = data_processor->path_radar4d[idx];

  auto& ds = ds_radar4d[idx];

  // /* ars_548
  if (type == Radar4dType::ARS548) {
    // auto msg_ptr = m.instantiate<ars548_msg::DetectionList>();
    auto msg_ptr = m.instantiate<ars548_process::DetectionList>();
    // std::cout << "instantiate failed type=" << m.getDataType() << std::endl;
    if (!msg_ptr) {
      return;
    }

    // const auto& detections = msg_ptr->detection_array;
    const auto& detections = msg_ptr->detections;

    if (!detections.empty()) {
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
        // Point.range_rate = detections[i].f_RangeRate * 100.0;
        // Point.range_rate_rms = detections[i].f_RangeRateSTD * 100.0;
        Point.range_rate = detections[i].f_range_rate * 100.0;
        Point.range_rate_rms = detections[i].f_range_rate_std * 100.0;
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
      // std::cout << "PointCloud.size : " << PointCloud.size() << std::endl;
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
        }

        for (int i = 0; i < PointCloud.data.size(); i++) {
          // clang-format off
          fprintf(fp_radar4d, "%f %f %f %f %f %f %f %f\n", 
                  float(PointCloud.data[i].x),       float(PointCloud.data[i].y),         float(PointCloud.data[i].z), 
                  float(PointCloud.data[i].range), 
                  float(PointCloud.data[i].azimuth), float(PointCloud.data[i].elevation), float(PointCloud.data[i].doppler),
                  float(PointCloud.data[i].rcs));
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

bool Ros1Convert::CheckSampledDataIsEnd() {
  auto check = [](const auto& vec) {
    for (const auto& ds : vec) {
      if (!ds.is_end) return false;
    }
    return true;
  };

  if (!check(ds_camera)) return false;
  if (!check(ds_infra)) return false;
  if (!check(ds_star)) return false;
  // if (!check(ds_radar4d)) return false;

  return true;
}

void Ros1Convert::LidarHandler(
    const sensor_msgs::PointCloud2::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    /* 
    std::cout << "PointCloud fields:" << std::endl;
    for (size_t i = 0; i < msg_ptr->fields.size(); ++i) {
      std::cout << msg_ptr->fields[i].name << std::endl;
    }
    abort();
    */

    uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;

    if (!data_processor->PushSampledTime(msg_time)) return;

    cloud_buffer_->clear();
    // 会按字段名自动匹配，并进行类型转换（如果可转换）
    // 不保证语义正确
    pcl::fromROSMsg(*msg_ptr, *cloud_buffer_);
#if defined(VELODYNE)
    VdToPcl(cloud_buffer_, cloud_);
    // std::cout << "vd_cloud_ptr convert suc " << std::endl;
#elif defined(USE_RSLIDAR)
    RsToPcl(cloud_buffer_, cloud_);
    // std::cout << "rs_cloud_ptr convert suc " << std::endl;
#elif defined(USE_LIVOX)
    LvToPcl(cloud_buffer_, cloud_);
    // std::cout << "rs_cloud_ptr convert suc " << std::endl;
#endif

    // warning
    pcl::transformPointCloud(*cloud_, *cloud_,
                             GetTransMatrix(rparam_->b_lt_none_rt));

    if (rparam_->b_save_data) {
      data_processor->SaveLidarData(cloud_, msg_time);
    }

    num_lidar_recv++;
  }
}

void Ros1Convert::LivoxLidarHandler(
    const livox_ros_driver2::CustomMsg::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;

    uint64 timebase = msg_ptr->timebase;

    // 只能手动执行转换，数据格式是 livox_ros_driver2::CustomMsg
    // 少一层拷贝
    // 参考  LvToPcl()
    cloud_->clear();

    size_t point_num = msg_ptr->point_num;
    if (point_num == 0) return;

    cloud_->points.reserve(point_num);
    for (size_t i = 0; i < point_num; i++) {
      const auto& pt = msg_ptr->points[i];

      bool invalid = !pcl::isFinite(pt) ||
                     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                     (pt.reflectivity == 0);
      if (invalid) continue;

      pcl::PointXYZIRT p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      // --- 自动兼容 ---
      p.intensity = pt.reflectivity;
      p.ring      = pt.line;
      // 基于 timebase 的偏移值：ns ==> ms
      // p.timestamp = static_cast<double>(pt.offset_time / 1000000.0);
      p.timestamp = static_cast<double>(pt.offset_time * 1e-6);
      cloud_->emplace_back(p);
    }

    // 不必再归一化时间戳 ==> (0, 100)
    // livox 频率为 10Hz，偏移值就属于 (0, 100)

    // warning
    pcl::transformPointCloud(*cloud_, *cloud_,
                             GetTransMatrix(rparam_->b_lt_none_rt));

    if (rparam_->b_save_data) {
      data_processor->SaveLidarData(cloud_, msg_time);
    }

    num_lidar_recv++;
  }
}

void Ros1Convert::NormalLidarHandler(
    const sensor_msgs::PointCloud2::ConstPtr& msg_ptr) {
  if (msg_ptr != NULL) {
    /* 
    std::cout << "PointCloud fields:" << std::endl;
    for (size_t i = 0; i < msg_ptr->fields.size(); ++i) {
      std::cout << msg_ptr->fields[i].name << std::endl;
    }
    abort();
    */

    uint64_t msg_time = msg_ptr->header.stamp.toSec() * 1000;

    if (!data_processor->PushSampledTime(msg_time)) return;

    cloud_->clear();
    pcl::fromROSMsg(*msg_ptr, *cloud_);

    pcl::transformPointCloud(*cloud_, *cloud_,
                             GetTransMatrix(rparam_->b_lt_none_rt));

    if (rparam_->b_save_data) {
      data_processor->SaveLidarData(cloud_, msg_time);
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

  cloud_buffer_->clear();
  pcl::fromROSMsg(msg, *cloud_buffer_);
#if defined(USE_RSLIDAR)
  RsToPcl(cloud_buffer_, cloud_);

  // std::cout << "point_rs_->width: " << cloud_buffer_->width << std::endl;
  // std::cout << "point_rs_->height: " << cloud_buffer_->height << std::endl;

  // show_pointcloud_ring(*cloud_buffer_);
  // show_pointcloud_strategy(*cloud_buffer_, ColorMode::RING);
  // show_pointcloud_height(*cloud_buffer_);
#endif

  // 去除 NaN 点
  // std::cout << "Before filter: " << cloud_->size() << " points" << std::endl;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud(
  //     new pcl::PointCloud<pcl::PointXYZI>);
  // std::vector<int> indices;  // 存储有效点的索引
  // pcl::removeNaNFromPointCloud(*cloud_, *FilteredCloud, indices);
  // std::cout << "After filter: " << FilteredCloud->size() << " points" << std::endl;

  // warning
  pcl::transformPointCloud(*cloud_, *cloud_,
                           GetTransMatrix(rparam_->b_lt_none_rt));

  if (rparam_->b_save_data) {
    data_processor->SaveLidarData(cloud_, msg_time);
  }

  num_lidar_recv++;
}

}  // namespace tools
}  // namespace jojo