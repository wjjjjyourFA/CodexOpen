#include "modules/perception/common/algorithm/point_cloud_processing/dbscan.h"

namespace jojo {
namespace perception {
namespace algorithm {

template <typename PointXYZV>
class DBSCAN_velocity : public DBSCAN<PointXYZV> {
  using DBSCAN::DBSCAN;

 public:
  // 这里应该叫做距离度量函数
  double Distance(const PointXYZV& a, const PointXYZV& b) {
    // 开方耗时，不开方能行吗
    double spatial_distance =
        std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                  (a.z - b.z) * (a.z - b.z));
    // 直接速度加减应该是有问题的
    double velocity_difference = std::abs(a.v - b.v);
    return spatial_distance + velocity_difference;
  };
};

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo