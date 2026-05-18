#ifndef __DBSCAN_H
#define __DBSCAN_H

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>

#pragma once

namespace jojo {
namespace perception {
namespace algorithm {

/* 有两种方法对点标签索引：
第一种，使用结构体，让其带有对应的子项。
第二种，在函数内部新建同样长度的索引数组。
这里选择第二种， 避免对点集重复操作。
*/
template <typename DataType>
class DBSCAN {
 public:
  // DBSCAN() {};
  virtual ~DBSCAN() = default;

  void set_params(double eps, int minPts) {
    this->eps_    = eps;
    this->minPts_ = minPts;
  };

  virtual void SetInputCloud(const std::vector<DataType> &cloud) {
    this->points_ = cloud;
    this->size_   = cloud.size();
  };

  // virtual void init() {};
  void Run() {
    // P1 —— P2 —— P3
    //      |
    //      P4

    // -1: 噪声, 0: 未分类, 其他: 簇 ID
    // 初始化：开始时，所有数据点均被标记为未分类
    // 使用构造函数初始化
    // this->labels_ = std::vector<int>(this->size_, 0);
    this->labels_.assign(this->size_, 0);
    this->cluster_id_ = 0;

    std::vector<int> neighbors;
    // 对每一个未分类的点，执行以下操作
    for (size_t i = 0; i < this->size_; ++i) {
      if (labels_[i] != 0) {
        continue;
      }

      // 计算每个点的邻域内的点
      neighbors.clear();
      RegionQuery(i, this->eps_, labels_, neighbors);

      // 判断是否是核心点，简单理解，看其邻域内的点数
      if (neighbors.size() < static_cast<size_t>(this->minPts_)) {
        labels_[i] = -1;  // Noise
      } else {
        ++cluster_id_;  // 增加类别数
        // 这里的 i 代表被选择的核心点
        ExpandClusterBfs(labels_, i, cluster_id_, neighbors);
      }
    }
  };

  virtual bool OutputCluster(std::vector<std::vector<DataType>> &clusters) {
    clusters.clear();
    /* way 1
    std::vector<std::vector<DataType>> tmp_clusters(cluster_id_);
    // 把不同的簇的点压进不同的vector当中
    for (size_t i = 0; i < this->size_; ++i) {
      if (labels_[i] > 0) {
        tmp_clusters[labels_[i] - 1].push_back(this->points_[i]);
      }
    }

    // 将tmp_clusters的结果拷贝到clusters中
    clusters.insert(clusters.end(), tmp_clusters.begin(), tmp_clusters.end());
    */

    // 调整clusters大小为cluster_id_
    clusters.resize(cluster_id_);
    // 把不同的簇的点压进不同的vector当中
    for (size_t i = 0; i < this->size_; ++i) {
      if (labels_[i] > 0) {
        clusters[labels_[i] - 1].push_back(this->points_[i]);
      }
    }

    return true;
  };

  // 此处为未排序或排序点集，全局遍历搜索
  virtual void RegionQuery(int index, double eps, std::vector<int> &labels,
                           std::vector<int> &neighbors) {
    for (size_t i = 0; i < this->size_; ++i) {
      // radius
      // 只跳过已分配到簇的点，允许未分类点和噪声点
      if (labels[i] > 0) {
        continue;
      }
      if (distance(this->points_[index], this->points_[i]) <= eps) {
        neighbors.push_back(i);
      }
    }
  };

  void ExpandClusterDfs(std::vector<int> &labels, int index, int cluster_id,
                        std::vector<int> &neighbors) {
    labels[index] = cluster_id;

    // 遍历所有邻居点
    for (int near_index : neighbors) {
      // 只处理未分类点或噪声点
      if (labels[near_index] > 0) {
        continue;
      }

      labels[near_index] = cluster_id;

      std::vector<int> new_neighbors;
      RegionQuery(near_index, this->eps_, labels, new_neighbors);

      // 如果该点是核心点（满足 MinPts），继续递归扩展
      if (new_neighbors.size() >= static_cast<size_t>(this->minPts_)) {
        ExpandClusterDfs(labels, near_index, cluster_id, new_neighbors);
        // 递归后，再合并 new_neighbors，确保完整性
        neighbors.insert(neighbors.end(), new_neighbors.begin(),
                         new_neighbors.end());
      }
    }
  };

  void ExpandClusterBfs(std::vector<int> &labels, int index, int cluster_id,
                        std::vector<int> &neighbors) {
    // 标记当前点的类别
    labels[index] = cluster_id;

    // BFS 广度优先搜索来扩展簇，而不是递归（避免栈溢出）。
    std::queue<int> process_queue;

    // 标记当前点的邻域内的点，并加入搜索队列
    for (int near_index : neighbors) {
      /* RegionQuery 已经处理过，这里不需要再处理
                            // 如果这个点已经分类（噪声点除外），就跳过
                            if (labels[near_index] > 0) {
                              continue;
                            } */
      labels[near_index] = cluster_id;
      process_queue.push(near_index);
    }

    while (!process_queue.empty()) {
      int current_point_index = process_queue.front();
      process_queue.pop();

      std::vector<int> new_neighbors;
      RegionQuery(current_point_index, this->eps_, labels, new_neighbors);

      if (new_neighbors.size() >= static_cast<size_t>(this->minPts_)) {
        for (int new_near_index : new_neighbors) {
          /* RegionQuery 已经处理过，这里不需要再处理
                                // 如果这个点已经分类（噪声点除外），就跳过
                                if (labels[new_near_index] > 0) {
                                  continue;
                                } */
          labels[new_near_index] = cluster_id;
          process_queue.push(new_near_index);
        }

        neighbors.insert(neighbors.end(), new_neighbors.begin(),
                         new_neighbors.end());
      }
    }
  };

  // 点集中每个点的标签
  std::vector<int> GetLabels() const { return labels_; }

 protected:
  int minPts_;  //最小聚类数
  double eps_;  //半径
  std::vector<DataType> points_;
  size_t size_;  // data_num
  std::vector<int> labels_;

 private:
  int cluster_id_ = 0;  // 数值代表分出几类点云簇

  // Dbscan 的复杂度一般是 O(n log n)（使用 KD-Tree 进行最近邻搜索），
  // 但如果直接用暴力搜索（两两计算距离），复杂度会是 O(n²)
  virtual double distance(const DataType &a, const DataType &b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
  };
};

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo

#endif