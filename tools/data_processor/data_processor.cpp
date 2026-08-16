#include "tools/data_processor/data_processor.h"

#include <cerrno>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <system_error>

#define foreach BOOST_FOREACH

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;
namespace camera = jojo::perception::camera;
namespace cfg    = jojo::perception::config;

DataProcessor::DataProcessor() {}

DataProcessor::~DataProcessor() { Stop(); }

bool DataProcessor::Init(std::shared_ptr<jojo::tools::RuntimeConfig> rparam,
                         std::shared_ptr<jojo::tools::InterfaceConfig> iparam) {
  if (rparam == nullptr || iparam == nullptr) {
    throw std::invalid_argument("DataProcessor configuration must not be null");
  }
  rparam_ = rparam;
  iparam_ = iparam;

  if (rparam_->b_do_undistort) {
    if (iparam_->b_camera || iparam_->b_infra || iparam_->b_star) {
      InitUndistortion();
    }
  }

  return true;
}

void DataProcessor::InitUndistortion() {
  camera_params = std::make_shared<camera::CameraParams>();
  camera_params->SetLoadPath(rparam_->calib_file_dir);

  for (int i = 0; i < iparam_->b_camera; i++) {
    camera_params->LoadFromName(rparam_->camera_name.at(i));

    // clang-format off
    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    // pinhole
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    // fisheye
    // camera_undistort->InitModel(camera::CameraDistortionModel::Kannala);
    // clang-format on
    undistort_vector.push_back(camera_undistort);
  }

  for (int i = 0; i < iparam_->b_infra; i++) {
    camera_params->LoadFromName(rparam_->infra_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
  }

  for (int i = 0; i < iparam_->b_star; i++) {
    camera_params->LoadFromName(rparam_->star_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
  }

  // clang-format off
  undistort_init.resize(iparam_->b_camera + iparam_->b_infra + iparam_->b_star, false);
  // clang-format on
}

void DataProcessor::Start() {
  sleep(1);
  if (!rparam_->b_save_data) {
    throw std::invalid_argument("DataProcessor requires b_save_data=true");
  }

  MkdirDataFolder();

  OpenWriteFile();
}

void DataProcessor::Stop() { CloseWriteFile(); }

std::vector<std::string> DataProcessor::MkdirDataFolderVector(
    const std::string& prefix, const std::string& name, int count) {
  std::vector<std::string> path;

  if (count <= 0) return path;

  /* 如果只有一个相机，就是 camera，否则是 camera_1, camera_2 ...
  if (count == 1) {
    std::string tmp = prefix + "/" + name;
    common::EnsureDirectory(tmp);
    path.push_back(tmp);
  } else {
    for (int i = 1; i <= count; i++) {
      std::string tmp = prefix + "/" + name + "_" + std::to_string(i);
      common::EnsureDirectory(tmp);
      path.push_back(tmp);
    }
  }
  */
  // 保持同样的组织格式
  for (int i = 1; i <= count; i++) {
    std::string tmp = prefix + "/" + name + "_" + std::to_string(i);
    common::EnsureDirectory(tmp);
    path.push_back(tmp);
  }

  return path;
}

void DataProcessor::MkdirDataFolder() {
  if (common::EnsureDirectory(rparam_->save_path)) {
    // clang-format off
    std::string time_str = FormatRosbagTime(rparam_->rosbag_name);
    // day 部分 (前 10 个字符)
    std::string day_str = time_str.substr(0, 10);
    this->prefix = rparam_->save_path + "/" + day_str;
    common::CreateDir(this->prefix);

    this->prefix = this->prefix + "/" + time_str;
    common::CreateDir(this->prefix);
    common::CreateDir(this->prefix + "/timestamp");

    // 输出路径
    this->postfix = this->prefix + "-O";
    common::CreateDir(this->postfix);

    if (iparam_->b_lidar){
      if (rparam_->use_bin_or_pcd == 0){
        path_lidar = this->prefix + "/" + "lidar";
      } else {
        path_lidar = this->prefix + "/" + "lidar_pcd";
        // path_lidar = this->prefix + "/" + "pcd";
      }
      common::CreateDir(path_lidar);
    }

    path_camera = MkdirDataFolderVector(this->prefix, "camera", iparam_->b_camera);
    path_camera_u = MkdirDataFolderVector(this->postfix, "sensor_data/camera_undistort", iparam_->b_camera);
    // std::cout << "path_camera_u.size() = " << path_camera_u.size() << std::endl;

    path_infra = MkdirDataFolderVector(this->prefix, "infra", iparam_->b_infra);
    path_infra_u = MkdirDataFolderVector(this->postfix, "sensor_data/infra_undistort", iparam_->b_infra);

    path_star = MkdirDataFolderVector(this->prefix, "star", iparam_->b_star);
    path_star_u = MkdirDataFolderVector(this->postfix, "sensor_data/star_undistort", iparam_->b_star);

    if (iparam_->b_radar){
      path_radar = this->prefix + "/" + "radar";
      common::CreateDir(path_radar);
    }

    path_radar4d = MkdirDataFolderVector(this->prefix, "radar4d", iparam_->b_radar4d);
    // clang-format on
  } else {
    std::cout << "CreateDir failed! " << std::endl;
    return;
  }
}

void DataProcessor::OpenWriteFile() {
  // 使用std::string和std::ofstream来处理文件路径和打开文件
  // 使用 fopen 打开文件，并赋值给 FILE* 类型的指针
  // clang-format off
  const auto open_file = [](const std::string& path) {
    FILE* file = std::fopen(path.c_str(), "w");
    if (file == nullptr) {
      throw std::system_error(errno, std::generic_category(), path);
    }
    return file;
  };

  try {
    if (iparam_->b_global_pose) {
      fp_global_pose = open_file(this->prefix + "/global_pose.txt");
    }
    if (iparam_->b_local_pose) {
      fp_local_pose = open_file(this->prefix + "/local_pose.txt");
    }
    // 你可以继续打开其他文件
    if (iparam_->b_imu_data) {
      fp_imu_data = open_file(this->prefix + "/imu_data.txt");
    }
  } catch (...) {
    CloseWriteFile();
    throw;
  }
  // clang-format on
}

void DataProcessor::CloseWriteFile() {
  // 确保文件指针被关闭
  const auto close_file = [](FILE*& file) {
    if (file != nullptr) {
      std::fclose(file);
      file = nullptr;
    }
  };
  close_file(fp_global_pose);
  close_file(fp_local_pose);
  close_file(fp_imu_data);
  // 继续关闭其他文件
}

void DataProcessor::SaveLidarData(pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud,
                                  uint64_t filename) {
  char buff[500];
  if (rparam_->use_bin_or_pcd == 0) {
    sprintf(buff, "%s/%013ld.bin", path_lidar.c_str(), filename);
    FILE* raw_file = std::fopen(buff, "wb");
    if (raw_file == nullptr) {
      std::cerr << "Lidar file create error: " << buff << ": "
                << std::strerror(errno) << std::endl;
      return;
    }
    const std::unique_ptr<FILE, decltype(&std::fclose)> fp_lidar(raw_file,
                                                                 &std::fclose);
    int tmp_x, tmp_y, tmp_z, tmp_intensity = 1;
    for (int i = 0; i < Cloud->points.size(); i++) {
      tmp_x         = int(Cloud->points[i].x * 100);
      tmp_y         = int(Cloud->points[i].y * 100);
      tmp_z         = int(Cloud->points[i].z * 100);
      tmp_intensity = int(Cloud->points[i].intensity);

      if (Cloud->points[i].x < -100 || Cloud->points[i].x > 100 ||
          Cloud->points[i].y < -100 || Cloud->points[i].y > 100 ||
          Cloud->points[i].z < -4 || Cloud->points[i].z > 8) {
        continue;
      }

      // 过滤掉全 0 行
      if (fabs(Cloud->points[i].x) < rparam_->distance_epsilon &&
          fabs(Cloud->points[i].y) < rparam_->distance_epsilon &&
          fabs(Cloud->points[i].z) < rparam_->distance_epsilon &&
          fabs(Cloud->points[i].intensity) < rparam_->intensity_epsilon) {
        continue;
      }

      const int point[] = {tmp_x, tmp_y, tmp_z, tmp_intensity};
      if (std::fwrite(point, sizeof(int), 4, fp_lidar.get()) != 4) {
        std::cerr << "Lidar file write error: " << buff << std::endl;
        return;
      }
      // std::cout << tmp_x << ", " << tmp_y << ", " << tmp_z << ", "
      //           << tmp_intensity << std::endl;
    }
  } else {
    sprintf(buff, "%s/%013ld.pcd", path_lidar.c_str(), filename);
    /*
    pcl::io::savePCDFileASCII(buff, Cloud);
    */
    pcl::io::savePCDFileBinary(buff, *Cloud);
  }
}

void DataProcessor::SaveLidarData(pcl::PointCloud<pcl::PointXYZIRT>::Ptr Cloud,
                                  uint64_t filename) {
  char buff[500];
  if (rparam_->use_bin_or_pcd == 0) {
    perror("Lidar PointCloud XYZIRT can't save as .bin file error!");
  } else {
    sprintf(buff, "%s/%013ld.pcd", path_lidar.c_str(), filename);
    pcl::io::savePCDFileBinary(buff, *Cloud);
  }
}

void DataProcessor::ProcessCameraImage(cv::Mat& image, uint64_t filename,
                                       const int& id, const int& mode) {
  int index = 0;
  std::string name;
  // 保存原始图像
  char file_image[300];
  switch (mode) {
      // clang-format off
    case 1:
      index = id;
      name  = "camera";
      if (rparam_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%013ld.jpg", path_camera.at(id).c_str(), filename);
      } else {
        sprintf(file_image, "%s/%013ld.png", path_camera.at(id).c_str(), filename);
      }
      break;

    case 2:
      index = iparam_->b_camera + id;
      name  = "infra";
      if (rparam_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%013ld.jpg", path_infra.at(id).c_str(), filename);
      } else {
        sprintf(file_image, "%s/%013ld.png", path_infra.at(id).c_str(), filename);
      }
      break;

    case 3:
      index = iparam_->b_camera + iparam_->b_infra + id;
      name  = "star";
      if (rparam_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%013ld.jpg", path_star.at(id).c_str(), filename);
      } else {
        sprintf(file_image, "%s/%013ld.png", path_star.at(id).c_str(), filename);
      }
      break;
      // clang-format on

    default:
      std::cout << "set mode error!" << std::endl;
      return;
  }

  if (rparam_->b_do_undistort) {
    if (index < 0 || index >= undistort_vector.size()) {
      std::cerr << "Invalid index " << index << " for undistort_vector size "
                << undistort_vector.size() << std::endl;
      return;
    }
  }
  // std::cout << file_image << std::endl;
  // cv::imwrite(file_image, image);
  cv::imwrite(file_image, image, rparam_->compress_params);

  // 去畸变
  if (rparam_->b_do_undistort) {
    // cv::Mat undistort_image(image.size(), CV_8UC3);  // 不用 zeros
    cv::Mat undistort_image = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
    if (!undistort_init.at(index)) {
      Eigen::VectorXf params(17);
      // 将所有元素置 0
      // params.setZero();

      /*  // way 1
      auto matrix = camera_params_vector.at(index)->GetMatrixVector();

      params = cfg::IntrinsicParamsToVector(
          matrix.at(0)->camera_matrix->intrinsic_matrix,
          matrix.at(0)->camera_matrix->distortion_params);
      */
      /*  way 2 // 不知道为什么 直接用一个参数类去管理所有参数 会跳出
      // 在这里 是设置临时变量，似乎有可以了？ 20250928
      auto &matrix = camera_params->GetMatrixVector().at(index);
      auto intrinsic_matrix =
          matrix_vector.at(index)->camera_matrix->intrinsic_matrix;
      auto distortion_params =
          matrix_vector.at(index)->camera_matrix->distortion_params;

      params = cfg::IntrinsicParamsToVector(intrinsic_matrix, distortion_params);
      */
      // /*
      auto& matrix_vector = camera_params->GetMatrixVector();
      auto& intrinsic_matrix =
          matrix_vector.at(index)->camera_matrix->intrinsic_matrix;
      auto& distortion_params =
          matrix_vector.at(index)->camera_matrix->distortion_params;

      params =
          cfg::IntrinsicParamsToVector(intrinsic_matrix, distortion_params);
      // */

      undistort_vector.at(index)->InitParams(image.cols, image.rows, params);
      undistort_vector.at(index)->Init(name);
      undistort_init.at(index) = true;
    }
    undistort_vector.at(index)->Handle(image, undistort_image);
    // std::cerr << " size : " << undistort_vector.size() << std::endl;

    char file_image_u[300];
    switch (mode) {
        // clang-format off
      case 1:
        if (rparam_->use_jpg_or_png <= 0) {
          sprintf(file_image_u, "%s/%013ld.jpg", path_camera_u.at(id).c_str(), filename);
        } else {
          sprintf(file_image_u, "%s/%013ld.png", path_camera_u.at(id).c_str(), filename);
        }
        break;

      case 2:
        if (rparam_->use_jpg_or_png <= 0) {
          sprintf(file_image_u, "%s/%013ld.jpg", path_infra_u.at(id).c_str(), filename);
        } else {
          sprintf(file_image_u, "%s/%013ld.png", path_infra_u.at(id).c_str(), filename);
        }
        break;

      case 3:
        if (rparam_->use_jpg_or_png <= 0) {
          sprintf(file_image_u, "%s/%013ld.jpg", path_star_u.at(id).c_str(), filename);
        } else {
          sprintf(file_image_u, "%s/%013ld.png", path_star_u.at(id).c_str(), filename);
        }
        break;
        // clang-format on

      default:
        std::cout << "set mode error!" << std::endl;
        break;
    }

    // cv::imwrite(file_image_u, undistort_image);
    cv::imwrite(file_image_u, undistort_image, rparam_->compress_params);
  }
}

DataProcessor::SampledTimeState DataProcessor::GetSampledTimeState(
    uint64_t msg_time, size_t sampled_index, int64_t& diff) const {
  if (rparam_->prepare_data_num == -1) {
    return SampledTimeState::kInWindow;
  }

  std::lock_guard<std::mutex> lock(sampled_time_mutex_);
  if (sampled_time.empty() || sampled_index >= sampled_time.size()) {
    return SampledTimeState::kFinished;
  }

  const uint64_t ref_time = sampled_time[sampled_index];
  if (msg_time >= ref_time) {
    const uint64_t delta = msg_time - ref_time;
    diff = delta > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())
               ? std::numeric_limits<int64_t>::max()
               : static_cast<int64_t>(delta);
  } else {
    const uint64_t delta = ref_time - msg_time;
    diff = delta > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())
               ? std::numeric_limits<int64_t>::min()
               : -static_cast<int64_t>(delta);
  }

  if (diff < -kSampledTimeToleranceMs) {
    // 太早
    return SampledTimeState::kTooEarly;
  }
  if (diff > kSampledTimeToleranceMs) {
    // 太晚
    return SampledTimeState::kTooLate;
  }
  return SampledTimeState::kInWindow;
}

bool DataProcessor::CheckSampledTime(uint64_t msg_time,
                                     size_t& sampled_index, int64_t& diff) {
  return GetSampledTimeState(msg_time, sampled_index, diff) ==
         SampledTimeState::kInWindow;
}

bool DataProcessor::PushSampledTime(uint64_t msg_time) {
  // for auto calib
  // 如果不需要采样，直接返回 true
  if (rparam_->prepare_data_num == -1) {
    return true;
  }

  std::lock_guard<std::mutex> lock(sampled_time_mutex_);
  if (b_final.load()) {
    return false;
  }

  if (b_first_grab) {
    start_time   = msg_time;
    b_first_grab = false;
  }

  // s ==> ms
  const uint64_t useless_time =
      static_cast<uint64_t>(rparam_->useless_time) * 1000U;
  if (msg_time < start_time || msg_time - start_time < useless_time) {
    return false;
  }

  b_grab = false;
  if (data_count % rparam_->sample_interval == 0) {
    sampled_time.push_back(msg_time);  // 采样
    b_grab = true;
    // std::cout<<"sampled_time: "<< msg_time <<std::endl;
  }
  data_count++;

  if (sampled_time.size() >=
      static_cast<size_t>(rparam_->prepare_data_num)) {
    b_final.store(true);
  }

  // 表示本次采样有效
  return b_grab;
}

bool DataProcessor::IsEnd(size_t& sampled_index) {
  std::lock_guard<std::mutex> lock(sampled_time_mutex_);
  return sampled_index >= sampled_time.size();
}

}  // namespace tools
}  // namespace jojo
