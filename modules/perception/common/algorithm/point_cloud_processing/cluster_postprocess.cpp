#include "modules/perception/common/algorithm/point_cloud_processing/cluster_postprocess.h"

namespace jojo {
namespace perception {
namespace algorithm {
namespace base = jojo::perception::base;

void CalculateSegAttributeLegacy(std::shared_ptr<base::Segment>& seg) {
  if (!seg->points || seg->points->empty()) {
    std::cerr << "CalculateSegAttributeLegacy: points is empty!" << std::endl;
    return;
  }

  // 使用 PCL 内置函数快速获取包络框
  pcl::PointXYZI min_pt, max_pt;  // 对应 seg->points 类型
  pcl::getMinMax3D(*seg->points, min_pt, max_pt);

  // 计算框中心
  seg->center = 0.5 * (min_pt.getVector3fMap() + max_pt.getVector3fMap());

  seg->size = Eigen::Vector3f(max_pt.x - min_pt.x, max_pt.y - min_pt.y,
                              max_pt.z - min_pt.z);

  // seg->bbox.p[2] = {min_pt.x, min_pt.y, min_pt.z};  // min corner
  // seg->bbox.p[4] = {max_pt.x, max_pt.y, max_pt.z};  // max corner
  seg->bbox = base::BBox3DExtra(min_pt.x, min_pt.y, min_pt.z, max_pt.x,
                                max_pt.y, max_pt.z);

  // 底部中心
  seg->anchor_point = Eigen::Vector3d(seg->center[0], seg->center[1], min_pt.z);
}

void CalculateSegAttribute(std::shared_ptr<base::Segment>& seg) {
  if (!seg->points || seg->points->empty()) {
    std::cerr << "CalculateSegAttribute: points is empty!" << std::endl;
    return;
  }

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();

  // 累加坐标用于质心计算
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();

  for (const auto& pt : *seg->points) {
    sum.x() += pt.x;
    sum.y() += pt.y;
    sum.z() += pt.z;

    /* way 1
    if (pt.x < min_x) min_x = pt.x;
    if (pt.x > max_x) max_x = pt.x;
    if (pt.y < min_y) min_y = pt.y;
    if (pt.y > max_y) max_y = pt.y;
    if (pt.z < min_z) min_z = pt.z;
    if (pt.z > max_z) max_z = pt.z;
    */
    // way 2
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
    min_z = std::min(min_z, pt.z);
    max_z = std::max(max_z, pt.z);
  }

  // 框中心
  const float center_x = (min_x + max_x) * 0.5f;
  const float center_y = (min_y + max_y) * 0.5f;
  const float center_z = (min_z + max_z) * 0.5f;
  seg->center = Eigen::Vector3f(center_x, center_y, center_z);

  const size_t point_count = seg->points->size();
  // 质心
  seg->centroid = sum / static_cast<double>(point_count);

  // 尺寸 length width height
  seg->size = Eigen::Vector3f(max_x - min_x, max_y - min_y, max_z - min_z);

  seg->bbox = base::BBox3DExtra(min_x, min_y, min_z, max_x, max_y, max_z);

  // 底部中心点
  seg->anchor_point = Eigen::Vector3d(center_x, center_y, min_z);
}

void GetLargestCluster(std::vector<std::shared_ptr<base::Segment>>& SegVector,
                       std::shared_ptr<base::Segment>& result) {
  size_t max_idx  = -1;
  size_t max_size = 0;

  // 计算有多少不同的簇，选取最大的簇
  for (size_t i = 0; i < SegVector.size(); ++i) {
    // 不同的簇
    auto& pts = SegVector[i]->points;
    if (!pts || pts->empty()) continue;

    if (pts->size() > max_size) {
      max_size = pts->size();
      max_idx  = i;
    }
  }

  if (max_idx != -1) {
    result = SegVector[max_idx];
    // std::cout << "GetLargestCluster: max_idx = " << max_idx
    //           << ", max_size = " << max_size << std::endl;
  } else {
    result = nullptr;  // 没有找到
    std::cout << "GetLargestCluster: no valid cluster found!" << std::endl;
  }
}

void GetClosestAmongTop2Clusters(
    std::vector<std::shared_ptr<base::Segment>>& SegVector,
    std::shared_ptr<base::Segment>& result) {
  // 最大第二大的两簇中里原点最近的一簇
  size_t max1_idx = -1, max2_idx = -1;
  size_t max1_size = 0, max2_size = 0;
  float max1_dist = std::numeric_limits<float>::max();
  float max2_dist = std::numeric_limits<float>::max();

  for (size_t i = 0; i < SegVector.size(); ++i) {
    auto& pts = SegVector[i]->points;
    if (!pts || pts->empty()) continue;

    size_t sz = pts->size();
    // 计算簇的框中心到原点距离平方
    auto& center = SegVector[i]->center;
    float dist2  = center.squaredNorm();

    if (sz > max1_size || (sz == max1_size && dist2 < max1_dist)) {
      // 当前簇变成最大簇，原最大簇降为第二大
      max2_idx  = max1_idx;
      max2_size = max1_size;
      max2_dist = max1_dist;

      max1_idx  = i;
      max1_size = sz;
      max1_dist = dist2;
    } else if (sz > max2_size || (sz == max2_size && dist2 < max2_dist)) {
      // 当前簇变成第二大簇
      max2_idx  = i;
      max2_size = sz;
      max2_dist = dist2;
    }
  }

  // 比较最大簇和第二大簇的距离，选择离原点最近的
  if (max1_idx == -1) result = nullptr;
  if (max2_idx == -1) result = SegVector[max1_idx];

  result = (max1_dist < max2_dist) ? SegVector[max1_idx] : SegVector[max2_idx];
}

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo