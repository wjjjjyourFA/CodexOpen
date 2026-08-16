#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

// #include <boost/foreach.hpp> // C++11 之前
#include <atomic>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include "cyber/common/file.h"
#include "modules/perception/common/camera/common/undistortion_handler.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/config/utils.h"
#include "modules/perception/common/lidar/convert/rs_sort_map.h"
#include "modules/perception/tools/pcl/pcl_viewer.h"
#include "modules/perception/tools/pcl/point_types.h"
#include "tools/common/utils/rosbag_utils.h"
#include "tools/data_processor/config/sensor_config.h"
#include "toolz/data_processor/config/interface_config.h"
#include "toolz/data_processor/config/runtime_config.h"

namespace jojo {
namespace tools {
using namespace std;

class DataProcessor {
 public:
  DataProcessor();  // Constructor
  virtual ~DataProcessor();

  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
            std::shared_ptr<jojo::tools::InterfaceConfig> interface);
  void InitUndistortion();

  void Start();
  void Stop();

  void SaveLidarData(pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud,
                     uint64_t filename);
  void SaveLidarData(pcl::PointCloud<pcl::PointXYZIRT>::Ptr Cloud,
                     uint64_t filename);

  void ProcessCameraImage(cv::Mat& image, uint64_t filename, const int& id,
                          const int& mode);

  static const int64_t kSampledTimeToleranceMs = 150;

  // 明确描述一帧数据相对当前雷达采样点的匹配状态。
  // sampled_index 的推进由调用方控制：只有提交或跳过当前采样点后才推进。
  enum class SampledTimeState {
    kTooEarly,  // 当前帧早于采样窗口
    kInWindow,  // 当前帧位于采样窗口内
    kTooLate,   // 当前帧晚于采样窗口
    kFinished,  // 没有待匹配的采样点
  };

  bool PushSampledTime(uint64_t msg_time);
  SampledTimeState GetSampledTimeState(uint64_t msg_time,
                                       size_t sampled_index,
                                       int64_t& diff) const;
  // 兼容旧转换流程；新代码应使用 GetSampledTimeState() 区分失败原因。
  bool CheckSampledTime(uint64_t msg_time, size_t& sampled_index,
                        int64_t& diff);
  std::atomic<bool> b_final{false};
  bool IsEnd(size_t& sampled_index);

  FILE* fp_global_pose{nullptr};
  FILE* fp_local_pose{nullptr};
  // 如果你需要其他文件流，也可以在这里声明
  FILE* fp_imu_data{nullptr};
  std::string path_radar;
  std::vector<std::string> path_radar4d;

 protected:
  std::vector<std::string> MkdirDataFolderVector(const std::string& prefix,
                                                 const std::string& name,
                                                 int count);
  void MkdirDataFolder();

  void OpenWriteFile();
  void CloseWriteFile();

  // clang-format off
  std::shared_ptr<jojo::perception::camera::CameraParams> camera_params;
  // std::vector<std::shared_ptr<jojo::perception::camera::CameraParams>> camera_params_vector;
  std::vector<std::shared_ptr<jojo::perception::camera::UndistortionHandler>> undistort_vector;
  std::vector<uint8_t> undistort_init;
  // clang-format on

  int data_count = 0;
  uint64_t start_time{0};
  std::vector<uint64_t> sampled_time;
  bool b_first_grab = true, b_grab = false;
  mutable std::mutex sampled_time_mutex_;

 private:
  std::shared_ptr<jojo::tools::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::tools::InterfaceConfig> iparam_;

  std::string prefix;
  std::string postfix;
  std::string path_lidar;
  std::vector<std::string> path_camera;
  std::vector<std::string> path_camera_u;
  std::vector<std::string> path_infra;
  std::vector<std::string> path_infra_u;
  std::vector<std::string> path_star;
  std::vector<std::string> path_star_u;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_H
