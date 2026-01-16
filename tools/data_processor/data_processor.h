#ifndef DATA_PROCESSOR_HH
#define DATA_PROCESSOR_HH

// #include <boost/foreach.hpp> // C++11 之前
#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <iostream>
#include <vector>
#include <unordered_set>

#include "cyber/common/file.h"
#include "modules/perception/common/base/pcl_extra/pcl_viewer.h"
#include "modules/perception/common/config/utils.h"
#include "modules/perception/common/camera/common/undistortion_handler_legacy.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/lidar/convert/rs_sort_map.h"
#include "tools/common/utils/rosbag_utils.h"

#include "tools/data_processor/config/runtime_config.h"
#include "tools/data_processor/config/sensor_config.h"

namespace jojo {
namespace tools {
using namespace std;

class DataProcessor {
 public:
  DataProcessor();  // Constructor
  virtual ~DataProcessor();

  void Init(std::shared_ptr<RuntimeConfig> param);
  void InitUndis();

  void Start();
  void Stop();

  void SaveLidarData(pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud,
                     uint64_t filename);

  void ProcessCameraImage(cv::Mat& image, uint64_t filename, const int& id,
                          const int& mode);

  bool PushSampledTime(uint64_t msg_time);
  bool CheckSampledTime(uint64_t msg_time, size_t& sampled_index,
                        int64_t& diff);
  bool b_final = false;
  bool IsEnd(size_t& sampled_index);

  FILE* fp_global_pose;
  FILE* fp_local_pose;
  // 如果你需要其他文件流，也可以在这里声明
  FILE* fp_imu_data;
  std::string path_radar;
  std::vector<std::string> path_radar_4d;

 protected:
  std::vector<std::string> MkdirDataFolderVector(const std::string& prefix,
                                                 const std::string& name,
                                                 int count);
  void MkdirDataFolder();

  void OpenWriteFile();
  void CloseWriteFile();

  int data_count = 0;
  uint64_t start_time;
  std::vector<uint64_t> sampled_time;
  bool b_first_grab = true, b_grab = false;

 private:
  std::shared_ptr<RuntimeConfig> param_ /*runtime_config*/;

  // clang-format off
  std::shared_ptr<jojo::perception::camera::CameraParams> camera_params;
  // std::vector<std::shared_ptr<jojo::perception::camera::CameraParams>> camera_params_vector;
  std::vector<std::shared_ptr<jojo::perception::camera::UndistortionHandler>> undistort_vector;
  std::vector<bool> undistort_init;
  // clang-format on

 private:
  std::string prefix;
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

#endif  // DataProcessor
