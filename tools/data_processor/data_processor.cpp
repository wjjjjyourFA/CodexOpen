#include "tools/data_processor/data_processor.h"

#define foreach BOOST_FOREACH

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;
namespace camera = jojo::perception::camera;
namespace cfg = jojo::perception::config;

DataProcessor::DataProcessor() {}

DataProcessor::~DataProcessor() {}

void DataProcessor::Init(std::shared_ptr<RuntimeConfig> param) {
  param_ = param;

  if (param_->b_camera || param_->b_infra || param_->b_star) {
    InitUndis();
  }
}

void DataProcessor::InitUndis() {
  camera_params = std::make_shared<camera::CameraParams>();
  camera_params->SetLoadPath(param_->calib_file_path);

  for (int i = 0; i < param_->b_camera; i++) {
    camera_params->LoadFromName(param_->camera_name.at(i));

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  for (int i = 0; i < param_->b_infra; i++) {
    camera_params->LoadFromName(param_->infra_name.at(i));

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  for (int i = 0; i < param_->b_star; i++) {
    camera_params->LoadFromName(param_->star_name.at(i));

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  // clang-format off
  undistort_init.resize(param_->b_camera + param_->b_infra + param_->b_star, false);
  // clang-format on
}

void DataProcessor::Start() {
  sleep(1);
  if (!param_->b_save_data) {
    std::cout << "b_save_data is false! " << std::endl;
    abort();
  }

  MkdirDataFolder();

  OpenWriteFile();
}

void DataProcessor::Stop() { CloseWriteFile(); }

std::vector<std::string> DataProcessor::MkdirDataFolderVector(
    const std::string& prefix, const std::string& name, int count) {
  std::vector<std::string> path;

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

  return path;
}

void DataProcessor::MkdirDataFolder() {
  if (common::EnsureDirectory(param_->save_path)) {
    // clang-format off
    std::string time_str = FormatRosbagTime(param_->rosbag_name);
    // day 部分 (前 10 个字符)
    std::string day_str = time_str.substr(0, 10);
    this->prefix = param_->save_path + "/" + day_str;
    common::CreateDir(this->prefix);

    this->prefix = this->prefix + "/" + time_str;
    common::CreateDir(this->prefix);

    if (param_->use_bin_or_pcd == 0){
      path_lidar = this->prefix + "/" + "lidar";
    } else {
      path_lidar = this->prefix + "/" + "lidar_pcd";
    }
    common::CreateDir(path_lidar);

    path_camera = MkdirDataFolderVector(this->prefix, "image", param_->b_camera);
    path_camera_u = MkdirDataFolderVector(this->prefix, "undistort_image", param_->b_camera);
    // std::cout << "path_camera_u.size() = " << path_camera_u.size() << std::endl;

    path_infra = MkdirDataFolderVector(this->prefix, "infra", param_->b_infra);
    path_infra_u = MkdirDataFolderVector(this->prefix, "undistort_infra", param_->b_infra);

    path_star = MkdirDataFolderVector(this->prefix, "star", param_->b_star);
    path_star_u = MkdirDataFolderVector(this->prefix, "undistort_star", param_->b_star);

    path_radar = this->prefix + "/" + "radar";
    common::CreateDir(path_radar);

    path_radar_4d = MkdirDataFolderVector(this->prefix, "radar_4d", param_->b_radar_4d);
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
  if (param_->b_global_pose) {
    std::string file_global_pose = this->prefix + "/" + "global_pose.txt";
    fp_global_pose = fopen(file_global_pose.c_str(), "w");
  }
  if (param_->b_local_pose) {
    std::string file_local_pose = this->prefix + "/" + "local_pose.txt";
    fp_local_pose = fopen(file_local_pose.c_str(), "w");
  }
  // 你可以继续打开其他文件
  if (param_->b_imu_data) {
    std::string file_imu_data = this->prefix + "/" + "imu_data.txt";
    fp_imu_data = fopen(file_imu_data.c_str(), "w");
  }
  // clang-format on
}

void DataProcessor::CloseWriteFile() {
  // 确保文件指针被关闭
  if (param_->b_global_pose && fp_global_pose != nullptr) {
    fclose(fp_global_pose);
  }
  if (param_->b_local_pose && fp_local_pose != nullptr) {
    fclose(fp_local_pose);
  }
  // 继续关闭其他文件
  if (param_->b_imu_data && fp_imu_data != nullptr) {
    fclose(fp_imu_data);
  }
}

void DataProcessor::SaveLidarData(pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud,
                                  uint64_t filename) {
  char buff[500];
  if (param_->use_bin_or_pcd == 0) {
    sprintf(buff, "%s/%ld.bin", path_lidar.c_str(), filename);
    FILE* fp_lidar;
    fp_lidar = fopen(buff, "wb");
    if (fp_lidar == NULL) {
      perror("Lidar file create error!");
    }
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
      if (fabs(Cloud->points[i].x) < param_->distance_epsilon &&
          fabs(Cloud->points[i].y) < param_->distance_epsilon &&
          fabs(Cloud->points[i].z) < param_->distance_epsilon &&
          fabs(Cloud->points[i].intensity) < param_->intensity_epsilon) {
        continue;
      }

      fwrite(&(tmp_x), sizeof(int), 1, fp_lidar);
      fwrite(&(tmp_y), sizeof(int), 1, fp_lidar);
      fwrite(&(tmp_z), sizeof(int), 1, fp_lidar);
      fwrite(&tmp_intensity, sizeof(int), 1, fp_lidar);
      // std::cout << tmp_x << ", " << tmp_y << ", " << tmp_z << ", "
      //           << tmp_intensity << std::endl;
    }
    fclose(fp_lidar);
  } else {
    sprintf(buff, "%s/%ld.pcd", path_lidar.c_str(), filename);
    /*
    pcl::io::savePCDFileASCII(buff, Cloud);
    */
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
    case 1:
      index = id;
      name  = "camera";
      if (param_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%ld.jpg", path_camera.at(id).c_str(), filename);
      } else {
        sprintf(file_image, "%s/%ld.png", path_camera.at(id).c_str(), filename);
      }
      break;

    case 2:
      index = param_->b_camera + id;
      name  = "infra";
      if (param_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%ld.jpg", path_infra.at(id).c_str(), filename);
      } else {
        sprintf(file_image, "%s/%ld.png", path_infra.at(id).c_str(), filename);
      }
      break;

    case 3:
      index = param_->b_camera + param_->b_infra + id;
      name  = "star";
      if (param_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%ld.jpg", path_star.at(id).c_str(), filename);
      } else {
        sprintf(file_image, "%s/%ld.png", path_star.at(id).c_str(), filename);
      }
      break;

    default:
      std::cout << "set mode error!" << std::endl;
      break;
  }

  if (index < 0 || index >= undistort_vector.size()) {
    std::cerr << "Invalid index " << index << " for undistort_vector size "
              << undistort_vector.size() << std::endl;
    return;
  }

  // std::cout << file_image << std::endl;
  // cv::imwrite(file_image, image);
  cv::imwrite(file_image, image, param_->compress_params);

  // 去畸变
  if (param_->b_undistort) {
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

    // clang-format off
    char file_image_u[300];
    switch (mode) {
      case 1:
        if (param_->use_jpg_or_png <= 0) {
          sprintf(file_image_u, "%s/%ld.jpg", path_camera_u.at(id).c_str(), filename);
        } else {
          sprintf(file_image_u, "%s/%ld.png", path_camera_u.at(id).c_str(), filename);
        }
        break;

      case 2:
        if (param_->use_jpg_or_png <= 0) {
          sprintf(file_image_u, "%s/%ld.jpg", path_infra_u.at(id).c_str(), filename);
        } else {
          sprintf(file_image_u, "%s/%ld.png", path_infra_u.at(id).c_str(), filename);
        }
        break;

      case 3:
        if (param_->use_jpg_or_png <= 0) {
          sprintf(file_image_u, "%s/%ld.jpg", path_star_u.at(id).c_str(), filename);
        } else {
          sprintf(file_image_u, "%s/%ld.png", path_star_u.at(id).c_str(), filename);
        }
        break;

      default:
        std::cout << "set mode error!" << std::endl;
        break;
    }
    // clang-format on

    // cv::imwrite(file_image_u, undistort_image);
    cv::imwrite(file_image_u, undistort_image, param_->compress_params);
  }
}

bool DataProcessor::CheckSampledTime(uint64_t msg_time, size_t& sampled_index,
                                     int64_t& diff) {
  if (param_->prepare_data_num == -1) {
    return true;
  }

  if (sampled_time.empty() || sampled_index >= sampled_time.size()) {
    return false;
  }

  static const int tolerance = 150;

  uint64_t ref_time = sampled_time[sampled_index];
  // std::cout << " " << ref_time << " " << msg_time << std::endl;
  diff = static_cast<int64_t>(msg_time) - static_cast<int64_t>(ref_time);

  if (diff < -tolerance) {
    // 太早
    return false;
  }

  if (diff > tolerance) {
    // 太晚，移除过期的时间戳
    // sampled_time.erase(sampled_time.begin());
    // 推进游标
    // ++sampled_index;
    return false;
  }

  // 在允许的范围内，命中消费掉这个时间点
  // 如果有 12 14 16 都像 15 逼近的话，会导致在 12 就把游标推进
  // 因此修改为由外部判断游标是否需要推进
  // ++sampled_index;
  return true;
}

bool DataProcessor::PushSampledTime(uint64_t msg_time) {
  // for auto calib
  // 如果不需要采样，直接返回 true
  if (param_->prepare_data_num == -1) {
    return true;
  }

  if (b_first_grab) {
    start_time   = msg_time;
    b_first_grab = false;
  }

  // s ==> ms
  const int useless_time = param_->useless_time * 1000;
  if (msg_time - start_time < useless_time) {
    return false;
  }

  b_grab = false;
  if (data_count % param_->sample_interval == 0) {
    sampled_time.push_back(msg_time);  // 采样
    b_grab = true;
    // std::cout<<"sampled_time: "<< msg_time <<std::endl;
  }
  data_count++;

  if (sampled_time.size() > param_->prepare_data_num) {
    b_final = true;
  }

  // 表示本次采样有效
  return b_grab && !b_final;
}

bool DataProcessor::IsEnd(size_t& sampled_index) {
  return sampled_index >= sampled_time.size();
}

}  // namespace tools
}  // namespace jojo
