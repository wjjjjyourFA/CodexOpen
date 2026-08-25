#ifndef MODULES_PERCEPTION_COMMON_ALGORITHM_POINT_CLOUD_PROCESSING_DBSCAN_H_
#define MODULES_PERCEPTION_COMMON_ALGORITHM_POINT_CLOUD_PROCESSING_DBSCAN_H_

#include <cmath>
#include <cstddef>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace jojo {
namespace perception {
namespace algorithm {

/* 有两种方法对点标签索引：
第一种，使用结构体，让其带有对应的子项。
第二种，在函数内部新建同样长度的索引数组。
这里选择第二种， 避免对点集重复操作。
*/

// Standard DBSCAN labels:
//   0  : unclassified
//   -1 : noise
//   >0 : cluster id
//
// min_pts includes the query point itself, as in the original DBSCAN definition.
// RegionQuery only performs a geometric epsilon-neighborhood query;
// point visitation and cluster assignment are handled separately.
template <typename DataType>
class DBSCAN {
 public:
  virtual ~DBSCAN() = default;

  bool set_params(double eps, int min_pts) {
    if (!std::isfinite(eps) || eps <= 0.0 || min_pts <= 0) {
      return false;
    }

    eps_    = eps;
    minPts_ = min_pts;
    ResetClusteringState();
    RebuildSpatialIndex(eps_);
    return true;
  }

  virtual void SetInputCloud(const std::vector<DataType>& cloud) {
    points_ = cloud;
    size_   = points_.size();
    ResetClusteringState();
    RebuildSpatialIndex(eps_);
  }

  bool Run() {
    ResetClusteringState();
    if (eps_ <= 0.0 || minPts_ <= 0 || points_.empty()) {
      return false;
    }

    std::vector<int> neighbors;
    for (size_t i = 0; i < size_; ++i) {
      // 对每一个未分类的点，执行以下操作
      if (visited_[i]) {
        continue;
      }

      // A point becomes visited when its neighborhood is queried,
      // not when it is assigned to a cluster.
      // This distinction is required for noise points to be absorbed later as border points.
      visited_[i] = true;
      // 计算每个点的邻域内的点
      RegionQuery(static_cast<int>(i), eps_, neighbors);

      // 判断是否是核心点，简单理解，看其邻域内的点数
      if (neighbors.size() < static_cast<size_t>(minPts_)) {
        labels_[i] = kNoise;
        continue;
      }

      // 增加类别数
      ++cluster_id_;
      // 这里的 i 代表被选择的核心点
      ExpandClusterBfs(labels_, static_cast<int>(i), cluster_id_, neighbors);
    }
    return true;
  }

  virtual bool OutputCluster(std::vector<std::vector<DataType>>& clusters) {
    clusters.clear();
    // 调整 clusters 数量为 cluster_id_ 的大小
    clusters.resize(static_cast<size_t>(cluster_id_));
    // 把不同的簇的点压进各自的 vector 当中
    for (size_t i = 0; i < size_; ++i) {
      if (labels_[i] > 0) {
        clusters[static_cast<size_t>(labels_[i] - 1)].push_back(points_[i]);
      }
    }
    return true;
  }

  // Returns every point in the closed epsilon-neighborhood, including the query point itself.
  // Cluster labels must not affect neighborhood density.
  virtual void RegionQuery(int index, double eps, std::vector<int>& neighbors) {
    neighbors.clear();
    if (index < 0 || static_cast<size_t>(index) >= size_ ||
        !std::isfinite(eps) || eps <= 0.0) {
      return;
    }

    // 使用空间索引加速查询，27个邻域网格，网格大小为 eps
    if (spatial_index_.empty() || indexed_eps_ != eps) {
      RebuildSpatialIndex(eps);
    }

    const CellKey center     = CellFor(points_[index], eps);
    const double eps_squared = eps * eps;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const CellKey key{center.x + dx, center.y + dy, center.z + dz};
          const auto cell = spatial_index_.find(key);
          if (cell == spatial_index_.end()) {
            continue;
          }

          for (const int candidate : cell->second) {
            // 相邻 Cell 不代表两个点一定在 eps 半径内，需要进一步判断
            if (SquaredDistance(points_[index], points_[candidate]) <=
                eps_squared) {
              neighbors.push_back(candidate);
            }
          }
        }
      }
    }
  }

  // Compatibility overload for existing callers.
  // Labels are intentionally ignored:  RegionQuery is a pure geometric query in standard DBSCAN.
  virtual void RegionQuery(int index, double eps, std::vector<int>& /*labels*/,
                           std::vector<int>& neighbors) {
    RegionQuery(index, eps, neighbors);
  }

  // Kept for source compatibility. The iterative implementation has the same
  // DBSCAN expansion semantics without risking recursion-stack overflow.
  void ExpandClusterDfs(std::vector<int>& labels, int index, int cluster_id,
                        std::vector<int>& neighbors) {
    ExpandClusterBfs(labels, index, cluster_id, neighbors);
  }

  void ExpandClusterBfs(std::vector<int>& labels, int index, int cluster_id,
                        std::vector<int>& neighbors) {
    if (index < 0 || static_cast<size_t>(index) >= size_) {
      return;
    }
    if (labels.size() != size_) {
      labels.assign(size_, kUnclassified);
    }
    if (visited_.size() != size_) {
      visited_.assign(size_, false);
    }

    labels[index] = cluster_id;

    // BFS 广度优先搜索来扩展簇，而不是递归（避免栈溢出）
    std::queue<int> process_queue;
    std::vector<bool> queued(size_, false);

    // 标记当前点的邻域内的点，并加入搜索队列
    for (const int neighbor : neighbors) {
      EnqueueIfNeeded(neighbor, queued, process_queue);
    }

    while (!process_queue.empty()) {
      const int current = process_queue.front();
      process_queue.pop();

      if (!visited_[current]) {
        visited_[current] = true;

        std::vector<int> current_neighbors;
        RegionQuery(current, eps_, current_neighbors);
        if (current_neighbors.size() >= static_cast<size_t>(minPts_)) {
          // Only core points expand the search frontier.
          // Border points are assigned to the cluster but do not propagate it further.
          for (const int neighbor : current_neighbors) {
            EnqueueIfNeeded(neighbor, queued, process_queue);
          }
        }
      }

      // A point previously marked as noise may become a border point.
      // Points already owned by another cluster are never reassigned.
      if (labels[current] == kUnclassified || labels[current] == kNoise) {
        labels[current] = cluster_id;
      }
    }
  }

  // 点集中每个点的标签
  std::vector<int> GetLabels() const { return labels_; }

 protected:
  int minPts_  = 0;  // 最小聚类数
  double eps_  = 0.0;  // 半径
  size_t size_ = 0;
  std::vector<DataType> points_;
  std::vector<int> labels_;
  std::vector<bool> visited_;

  // 子类只需重写该距离度量函数，即可复用完整的 DBSCAN 流程。
  // 返回距离的平方，以避免基础欧氏距离执行不必要的开方。
  // 空间哈希仍按 XYZ 做候选点预筛选，因此自定义度量不应把 XYZ
  // 距离大于 eps 的点判定为邻居。
  virtual double SquaredDistance(const DataType& a,
                                 const DataType& b) const {
    const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
    const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
    const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
    return dx * dx + dy * dy + dz * dz;
  }

 private:
  static constexpr int kNoise        = -1;
  static constexpr int kUnclassified = 0;

  struct CellKey {
    int x;
    int y;
    int z;

    bool operator==(const CellKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct CellKeyHash {
    size_t operator()(const CellKey& key) const {
      size_t seed = std::hash<int>{}(key.x);
      seed ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  CellKey CellFor(const DataType& point, double cell_size) const {
    return {static_cast<int>(std::floor(point.x / cell_size)),
            static_cast<int>(std::floor(point.y / cell_size)),
            static_cast<int>(std::floor(point.z / cell_size))};
  }

  void ResetClusteringState() {
    labels_.assign(size_, kUnclassified);
    visited_.assign(size_, false);
    cluster_id_ = 0;
  }

  void RebuildSpatialIndex(double cell_size) {
    spatial_index_.clear();
    indexed_eps_ = cell_size;
    if (!std::isfinite(cell_size) || cell_size <= 0.0) {
      return;
    }

    spatial_index_.reserve(points_.size());
    for (size_t i = 0; i < points_.size(); ++i) {
      spatial_index_[CellFor(points_[i], cell_size)].push_back(
          static_cast<int>(i));
    }
  }

  void EnqueueIfNeeded(int index, std::vector<bool>& queued,
                       std::queue<int>& process_queue) const {
    if (index < 0 || static_cast<size_t>(index) >= size_ || queued[index]) {
      return;
    }
    queued[index] = true;
    process_queue.push(index);
  }

  int cluster_id_     = 0;  // 数值代表分出几类点云簇
  double indexed_eps_ = 0.0;
  std::unordered_map<CellKey, std::vector<int>, CellKeyHash> spatial_index_;

  // Dbscan 的复杂度一般是 O(n log n)（使用 KD-Tree 进行最近邻搜索），但如果直接用暴力搜索（两两计算距离），复杂度会是 O(n²)
};

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo

#endif  // MODULES_PERCEPTION_COMMON_ALGORITHM_POINT_CLOUD_PROCESSING_DBSCAN_H_
