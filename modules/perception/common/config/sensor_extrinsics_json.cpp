#include "modules/perception/common/config/sensor_extrinsics_json.h"

namespace jojo {
namespace perception {
namespace config {
using json = nlohmann::json;

bool SensorExtrinsicsJson::LoadFromFileBase(const std::string& filename,
                                            Eigen::Matrix4f& extrinsic_matrix) {
  std::ifstream fin2(filename);
  if (fin2.is_open() != 1) {
    std::cerr << "Fail to open params file: " << filename << std::endl;
    abort();
    return false;
  }
}

}  // namespace config
}  // namespace perception
}  // namespace jojo
