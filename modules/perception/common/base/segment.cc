#include "modules/perception/common/base/segment.h"

namespace jojo {
namespace perception {
namespace base {

Segment::Segment() {
  points =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

void Segment::Reset() {
  id = -1;
  // polygon.clear();
  bbox = BBox3DExtra();
  center.setZero();
  centroid.setZero();
  size.setZero();
  anchor_point.setZero();
  if (points) points->clear();  // 清空点云，但保留分配好的内存
  points_index_vector.clear();
}

std::string Segment::ToString() const {
  std::ostringstream oss;
  // clang-format off
  oss << "Segment [id: " << id
      << ", center: (" << center[0] << "," << center[1] << "," << center[2]
      << "), size: (" << size[0] << "," << size[1] << "," << size[2]
      << "), anchor_point: (" << anchor_point[0]
      << "," << anchor_point[1] << "," << anchor_point[2]
      << ")";
  // clang-format on
  return oss.str();
}

}  // namespace base
}  // namespace perception
}  // namespace jojo
