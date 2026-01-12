#include "modules/perception/common/config/sensor_extrinsics.h"

namespace jojo {
namespace perception {
namespace config {

void SensorExtrinsics::SetLoadPath(const std::string& load_path) {
  LoadPath = load_path;
}

void SensorExtrinsics::LoadFromName(const std::string& sensor_name) {
  std::string sensor_file = LoadPath + "/" + sensor_name + ".ini";

  int sensor_p_num = extrinsics_.size() + 1;
  std::cout << ">>>>>>>> " << sensor_name << "_" << sensor_p_num << " : "
            << std::endl;

  ReadFileWrapper(sensor_file);
}

void SensorExtrinsics::LoadFromFile(const std::string& sensor_file) {
  int sensor_p_num = extrinsics_.size() + 1;
  std::cout << ">>>>>>>> " << "sensor_" << sensor_p_num << " : " << std::endl;

  ReadFileWrapper(sensor_file);
}

bool SensorExtrinsics::LoadFromFileBase(const char* filename,
                            Eigen::Matrix4f& extrinsic_matrix) {
  std::ifstream fin2(filename);
  if (fin2.is_open() != 1) {
    std::cerr << "Fail to open params file: " << filename << std::endl;
    abort();
    return false;
  }

  // default use m
  bool b_mm_or_m = 1;
  // clang-format off
  std::string t_s2;
  while (fin2 >> t_s2) {
    if (t_s2[0] == '#' || t_s2[0] == '/') {
      getline(fin2, t_s2);
    } else if (t_s2 == "Rotate") {
      fin2 >> extrinsic_matrix(0, 0) >> extrinsic_matrix(0, 1) >> extrinsic_matrix(0, 2) >> 
              extrinsic_matrix(1, 0) >> extrinsic_matrix(1, 1) >> extrinsic_matrix(1, 2) >>
              extrinsic_matrix(2, 0) >> extrinsic_matrix(2, 1) >> extrinsic_matrix(2, 2);
    } else if (t_s2 == "Translate") {
      fin2 >> extrinsic_matrix(0, 3) >> extrinsic_matrix(1, 3) >> extrinsic_matrix(2, 3);
    } else if (t_s2 == "b_mm_or_m") {
      fin2 >> b_mm_or_m;
    }
  }
  fin2.close();
  // clang-format on

  // Set the last row for RT and P matrices
  extrinsic_matrix(3, 0) = 0;
  extrinsic_matrix(3, 1) = 0;
  extrinsic_matrix(3, 2) = 0;
  extrinsic_matrix(3, 3) = 1;

  // mm2m
  if (b_mm_or_m == 0) {
    extrinsic_matrix = jojo::perception::config::TransRtMatrixmm2m(extrinsic_matrix);
  }

  return true;
}

bool SensorExtrinsics::ReadFileWrapper(const std::string& sensor_file) {
  auto transf_i = std::make_shared<ExtrinsicMatrix>();

  auto extrinsic_matrix = &transf_i->matrix;

  char filename[256];
  std::snprintf(filename, sizeof(filename), "%s", sensor_file.c_str());

  if (!LoadFromFileBase(filename, *extrinsic_matrix)) {
    std::cerr << "Fail to open params file: " << filename << std::endl;
    exit(EXIT_FAILURE);
    return false;
  }

  std::cout << "Extrinsic (RT) parameters are: " << std::endl
            << *extrinsic_matrix << std::endl
            << std::endl;

  extrinsics_.push_back(transf_i);

  return true;
}

}  // namespace config
}  // namespace perception
}  // namespace jojo
