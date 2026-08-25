#ifndef MODULES_PERCEPTION_COMMON_ALGORITHM_POINT_CLOUD_PROCESSING_DBSCAN_EXTENSIONS_H_
#define MODULES_PERCEPTION_COMMON_ALGORITHM_POINT_CLOUD_PROCESSING_DBSCAN_EXTENSIONS_H_

#include "modules/perception/common/algorithm/point_cloud_processing/dbscan.h"

namespace jojo {
namespace perception {
namespace algorithm {

// DBSCAN extension using a combined XYZ and velocity distance metric.
template <typename PointXYZV>
class DBSCAN_velocity : public DBSCAN<PointXYZV> {
 public:
  using DBSCAN<PointXYZV>::DBSCAN;

 protected:
  double SquaredDistance(const PointXYZV& a,
                         const PointXYZV& b) const override {
    const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
    const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
    const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
    const double dv = static_cast<double>(a.v) - static_cast<double>(b.v);

    // 位置和速度的单位不同，可通过权重调整速度差对聚类的影响。
    return dx * dx + dy * dy + dz * dz + kVelocityWeight * dv * dv;
  }

 private:
  static constexpr double kVelocityWeight = 1.0;
};

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo

#endif  // MODULES_PERCEPTION_COMMON_ALGORITHM_POINT_CLOUD_PROCESSING_DBSCAN_EXTENSIONS_H_
