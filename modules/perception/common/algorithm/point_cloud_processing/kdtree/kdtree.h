#ifndef __KD_TREE_FH_H__
#define __KD_TREE_FH_H__

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "modules/common/math/math_utils.h"
// #include "modules/perception/common/base/point.h"

namespace jojo {
namespace perception {
namespace algorithm {
// namespace math = jojo::common::math;
// namespace base = jojo::perception::base;

//
// KD Tree Base Class
//

typedef struct {
  float lower, upper;
} interval;

enum class SplitMethod {
  MEDIAN,  // 使用中位数
  MEAN,  // 使用均值
  MIDPOINT  // 使用边界中心
};

class KDTreeNode;
class KDTreeResultVector;
class SearchRecord;

// 使用索引方式构建 KD 树 在大数据集时更快
class KDTree {
 public:
  KDTree();
  ~KDTree();

  void init(float* data_in, int dim_in, int max_sample_size, bool sort_results);
  void build_tree(int num_in);

  void FreeTree();

  void radius_search(std::vector<float>& qv, float r2,
                     KDTreeResultVector& result);
  // search for all neighbors in ball of size (square Euclidean distance)
  // r2. Return number of neighbors in 'result.size()',
  void radius_search_around_point(int idx_in, int index_range, float r2,
                                  KDTreeResultVector& result);
  int r_count(std::vector<float>& qv, float r2);
  int r_count_around_point(int idxin, int correltime, float r2);

  void knn_search(std::vector<float>& qv, int nn, KDTreeResultVector& result);
  // default ball_size == infinity
  void knn_search_around_point(int idx_in, int index_range, int nn,
                               KDTreeResultVector& result);
  void knn_search_brute_force(std::vector<float>& qv, int nn,
                              KDTreeResultVector& result);

 protected:
  // 这是一块外部传入的内存，在构建 KD 树时，会直接使用这块内存，而不是重新分配内存。
  // 对于实际数据的操作是通过索引进行的，直接指向这块内存。
  float* buff_;
  int max_sample_size_;
  int* indices_;  // 指向数据的索引

  SplitMethod split_method_ = SplitMethod::MEDIAN;

 private:
  int dim_;
  bool sort_results_;  // sorting result?
  KDTreeNode* root_ = nullptr;  // the root pointer

  int data_num_;  // number of data points

  static const int bucket_size = 12;  // 叶子节点中存储的点数量

  virtual void build_tree() {}  // builds the tree.  Used upon construction.
  KDTreeNode* build_tree_for_range(int l, int u, KDTreeNode* parent);

  void spread_in_coordinate_box(int c, int l, int u, interval& interv);
  // 适用于 寻找中位数（Median）作为切分点
  void select_on_coordinate_index(int c, int k, int l, int u);
  // 适用于 基于数值阈值的切分
  int select_on_coordinate_value(int c, float alpha, int l, int u);

  void search(KDTreeNode* node, SearchRecord& sr);
  // recursive innermost core routine for searching..

  bool box_in_search_range(KDTreeNode* node, SearchRecord& sr);
  // return true if the bounding box for this node is within the
  // search range given by the searchvector and maximum ballsize in 'sr'.
  inline float distance_from_boundary(float x, float amin, float amax) {
    /* way 1
    if (x > amax) {
      return (x - amax);
    } else if (x < amin) {
      return (amin - x);
    } else {
      return 0.0;
    }
    */
    return std::max(0.0f, std::max(amin - x, x - amax));
  }

  // for processing final buckets.
  void process_terminal_node_knn(KDTreeNode* node, SearchRecord& sr);
  void process_terminal_node_fixedball(KDTreeNode* node, SearchRecord& sr);

  void check_query_in_bound(SearchRecord& sr);  // debugging only

  friend class KDTreeNode;
  friend class SearchRecord;
};

class KDTreeNode {
 public:
  KDTreeNode() {
    box  = std::vector<interval>(dim_);
    left = right = NULL;
  }
  KDTreeNode(int dim) : box(dim) {
    left = right = NULL;
    dim_         = dim;
  }
  ~KDTreeNode() {
    if (left != NULL) delete left;
    if (right != NULL) delete right;
  }

 private:
  int dim_ = 3;  // default x,y,z 3维度
  KDTreeNode *left, *right;  // 左、右子树
  // lower bound（左边界）：表示当前 KD 树节点负责的点集合的起始索引。
  // upper bound（右边界）：表示当前 KD 树节点负责的点集合的结束索引。
  int lower, upper;
  int cut_dim;  // dimension to cut;
  // cut_val: 表示当前 KD 树节点在切分维度上的切分值。
  // cut_val_left、right: 表示当前 KD 树节点在切分维度上的左、右边界值。
  float cut_val, cut_val_left, cut_val_right;  //cut value
  std::vector<interval> box;  // [min,max] of the box enclosing all points

  friend class KDTree;  // allow kdtree to access private data
  friend class SearchRecord;

  // 存储点
  // PointF point;
};

//
//  SearchRecord Base Class
//

static const float k_infinity = 1.0e38;

struct KDTreeResult {
 public:
  float dis;  // square distance
  int idx;  // neighbor index

  // for priority_queue
  bool operator<(const KDTreeResult& other) const {
    // 返回 true：表示 a 的优先级比 b 低（即 a.dis < b.dis 时，b 应该排在前面）。
    // 结果：最大 dis 在 results_[0]（即堆顶）。
    return dis > other.dis;  // 使 priority_queue 维护最大堆
  }
};

// ==> std::priority_queue<KDTreeResult> KDTreeResultVector;
class KDTreeResultVector {
  // 维护最近邻的最大 dis
  // 需要 动态插入新点，保持最大堆
  // 需要 快速找到最大距离值
 public:
  KDTreeResultVector() {}
  ~KDTreeResultVector() {}
  std::vector<KDTreeResult> data;

  // inherit a std::vector<KDTreeResult>
  // but, optionally maintain it in heap form as a priority
  // queue.
 public:
  // add one new element to the list of results, and
  // keep it in heap order.  To keep it in ordinary, as inserted,
  // order, then simply use push_back() as inherited
  // via std::vector<>
  void push_element_and_heapify(KDTreeResult& e) {
    data.push_back(e);  // what a vector does.
    // push_heap 的默认比较规则是 <
    std::push_heap(data.begin(),
                   data.end());  // and now heapify it, with the new elt.
  }

  // for knn
  float replace_maxpri_elt_return_new_maxpri(KDTreeResult& e) {
    // remove the maximum priority element on the queue and replace it
    // with 'e', and return its priority.
    // here, it means replacing the first element [0] with e, and re heapifying.
    if (data.empty()) return -1;  // 处理空堆情况

    // 先比较新元素与堆顶元素的优先级（即距离）
    if (e.dis < data.front().dis) {
      // 新点的距离更小，替换堆顶元素
      data.front() = e;
      std::make_heap(data.begin(), data.end());  // 重新堆化
    }

    return (data.front().dis);  // return the new maximum priority.
  }

  float max_value() const {
    // return ((*data.begin()).dis);  // very first element
    return data.empty() ? -1 : data.front().dis;
  }
  // return the distance which has the maximum value of all on list,
  // assuming that ALL insertions were made by
  // push_element_and_heapify()
};

class SearchRecord {
 public:
  SearchRecord(std::vector<float>& qv_in, KDTree& tree_in /*树指针*/,
               KDTreeResultVector& result_in)
      : qv(qv_in),
        result(result_in),
        buff_(tree_in.buff_),
        indices_(tree_in.indices_) {
    dim       = tree_in.dim_;
    ball_size = k_infinity;
    nn        = 0;
  }

 public:
  float* buff_;
  int* indices_;

 private:
  int dim;
  std::vector<float>& qv;  // query vector
  // float *qv;

  unsigned int nn;  // nfound;
  float ball_size;
  int center_idx;
  // 表示某种相关性，优化搜索过程
  // 如果一个密集点云被用于查找，那么一个点符合要求时，它附近的点也符合要求。
  int index_range = 1;
  KDTreeResultVector& result;  // results

  friend class KDTree;
  friend class KDTreeNode;
};

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo

#endif  // __KD_TREE_FH_H
