#include "modules/perception/common/camera/params/camera_params.h"

namespace jojo {
namespace perception {
namespace camera {

void CameraParams::SetLoadPath(const std::string& load_path) {
  LoadPath = load_path;
}

void CameraParams::LoadFromName(const std::string& camera_name) {
  std::string camera_file = LoadPath + "/" + camera_name + ".ini";

  std::cout << ">>>>>>>> " << camera_name << " : " << std::endl;

  ReadFileWrapper(camera_file);

  // std::cerr << " size : " << lidar2camera_vector.size() << std::endl;
}

void CameraParams::LoadFromFile(const std::string& camera_file) {
  int camera_p_num = lidar2camera_vector.size() + 1;
  std::cout << ">>>>>>>> " << "camera_" << camera_p_num << " : " << std::endl;

  ReadFileWrapper(camera_file);
}

bool CameraParams::LoadFromFileBase(
    const char* filename, Eigen::Matrix3f& intrinsic_matrix,
    Eigen::Matrix<float, 8, 1>& distortion_params,
    Eigen::Matrix4f& extrinsic_matrix, Eigen::Matrix4f& projection_matrix) {
  std::ifstream fin2(filename);
  if (fin2.is_open() != 1) {
    std::cerr << "Fail to open params file: " << filename << std::endl;
    abort();
    return false;
  }

  // default use P mm ==> m
  bool b_rt_or_p = 1;
  bool b_mm_or_m = 0;
  // clang-format off
  std::string t_s2;
  while (fin2 >> t_s2) {
    if (t_s2[0] == '#' || t_s2[0] == '/') {
      getline(fin2, t_s2);
    } else if (t_s2 == "K") {
      fin2 >> intrinsic_matrix(0, 0) >> intrinsic_matrix(0, 1) >> intrinsic_matrix(0, 2) >> 
              intrinsic_matrix(1, 0) >> intrinsic_matrix(1, 1) >> intrinsic_matrix(1, 2) >>
              intrinsic_matrix(2, 0) >> intrinsic_matrix(2, 1) >> intrinsic_matrix(2, 2);
    } else if (t_s2 == "Rotate") {
      fin2 >> extrinsic_matrix(0, 0) >> extrinsic_matrix(0, 1) >> extrinsic_matrix(0, 2) >> 
              extrinsic_matrix(1, 0) >> extrinsic_matrix(1, 1) >> extrinsic_matrix(1, 2) >>
              extrinsic_matrix(2, 0) >> extrinsic_matrix(2, 1) >> extrinsic_matrix(2, 2);
    } else if (t_s2 == "Translate") {
      fin2 >> extrinsic_matrix(0, 3) >> extrinsic_matrix(1, 3) >> extrinsic_matrix(2, 3);

    } else if (t_s2 == "k1k2k3") {
      fin2 >> distortion_params(0) >> distortion_params(1) >> distortion_params(4);

    } else if (t_s2 == "k1k2k3k4") {
      fin2 >> distortion_params(0) >> distortion_params(1) >> distortion_params(2) >> distortion_params(3);

    } else if (t_s2 == "p1p2") {
      fin2 >> distortion_params(2) >> distortion_params(3);

    } else if (t_s2 == "P") {
      fin2 >> projection_matrix(0, 0) >> projection_matrix(0, 1) >> projection_matrix(0, 2) >> projection_matrix(0, 3) >>
              projection_matrix(1, 0) >> projection_matrix(1, 1) >> projection_matrix(1, 2) >> projection_matrix(1, 3) >>
              projection_matrix(2, 0) >> projection_matrix(2, 1) >> projection_matrix(2, 2) >> projection_matrix(2, 3);
    } else if (t_s2 == "b_rt_or_p") {
      fin2 >> b_rt_or_p;
    } else if (t_s2 == "b_mm_or_m") {
      fin2 >> b_mm_or_m;
    }
  }
  fin2.close();
  // clang-format on

  // Set the last row for RT and P matrices
  extrinsic_matrix(3, 0)  = 0;
  extrinsic_matrix(3, 1)  = 0;
  extrinsic_matrix(3, 2)  = 0;
  extrinsic_matrix(3, 3)  = 1;
  projection_matrix(3, 0) = 0;
  projection_matrix(3, 1) = 0;
  projection_matrix(3, 2) = 0;
  projection_matrix(3, 3) = 1;

  // If the flag b_rt_or_p is 0, compute the projection matrix as K * RT
  if (b_rt_or_p == 0) {
    projection_matrix.block<3, 4>(0, 0) =
        intrinsic_matrix * extrinsic_matrix.block<3, 4>(0, 0);
  } else {
    extrinsic_matrix =
        config::ComputeExtrinsicMatrix(intrinsic_matrix, projection_matrix);
  }

  // mm2m
  if (b_mm_or_m == 0) {
    projection_matrix =
        config::TransProjMatrixmm2m(intrinsic_matrix, projection_matrix);
    extrinsic_matrix = config::TransRtMatrixmm2m(extrinsic_matrix);
  }

  return true;
}

// 20240508 LM used Front Left Up
bool CameraParams::ReadFileWrapper(const std::string& camera_file) {
  char filename[256];
  std::snprintf(filename, sizeof(filename), "%s", camera_file.c_str());

  auto camera_matrix_i = std::make_shared<CameraMatrix>();
  auto lidar2camera_i  = std::make_shared<Lidar2CameraMatrix>();

  auto& intrinsic_matrix  = camera_matrix_i->intrinsic_matrix;
  auto& distortion_params = camera_matrix_i->distortion_params;
  auto& extrinsic_matrix  = lidar2camera_i->extrinsic_matrix;
  auto& projection_matrix = lidar2camera_i->projection_matrix;

  if (!LoadFromFileBase(filename, intrinsic_matrix, distortion_params,
                        extrinsic_matrix, projection_matrix)) {
    std::cerr << "Fail to open params file: " << filename << std::endl;
    exit(EXIT_FAILURE);
    return false;
  }

  lidar2camera_i->camera_matrix = camera_matrix_i;

  std::cout << "K parameters are: " << std::endl
            << intrinsic_matrix << std::endl
            << std::endl;

  std::cout << "Distortion parameters are: " << std::endl
            << distortion_params << std::endl
            << std::endl;

  std::cout << "Extrinsic (RT) parameters are: " << std::endl
            << extrinsic_matrix << std::endl
            << std::endl;

  std::cout << "Projection matrix (P) parameters are: " << std::endl
            << projection_matrix << std::endl
            << std::endl;

  camera_vector.push_back(camera_matrix_i);
  lidar2camera_vector.push_back(lidar2camera_i);
  // std::cerr << " size : " << lidar2camera_vector.size() << std::endl;

  return true;
}

bool CameraParams::InitMatrixVector(int camera_p_num) {
  camera_vector.resize(camera_p_num);

  // 确保 vector 至少有 i 个元素
  lidar2camera_vector.resize(camera_p_num);
  camera_vector.resize(camera_p_num);
  /*
  for (int i = 0; i < camera_p_num; ++i) {
      lidar2camera_vector[i] = std::make_shared<Lidar2CameraMatrix>();
      camera_vector[i]       = std::make_shared<CameraMatrix>();
  }
  */
  // std::cout << "lidar2camera_vector.size(): " << lidar2camera_vector.size()
  //           << std::endl;

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace jojo
