#include "modules/perception/common/algorithm/point_cloud_processing/kdtree/kdtree.h"

namespace jojo {
namespace perception {
namespace algorithm {

KDTree::KDTree() {}

KDTree::~KDTree() {
  // Here you would include logic to delete all nodes to prevent memory leaks
  delete root_;
  delete indices_;
}

void KDTree::init(float* data_in, int dim_in, int max_sample_size,
                  bool sort_results) {
  // Set member variables
  buff_            = data_in;  // data buffer，实际数据小于等于buff_的大小
  dim_             = dim_in;  // data dimension
  max_sample_size_ = max_sample_size;
  sort_results_    = sort_results;

  // Allocate memory for indices
  indices_ = new int[max_sample_size];
  // 是否可以提前分配索引值，避免重复分配
}

void KDTree::build_tree(int num_in) {
  // 动态初始化，避免重复分配内存
  this->FreeTree();

  // 依据实际数据构建树
  data_num_ = num_in;

  // Debug
  // num_of_nodes = 0;

  for (int i = 0; i < data_num_; i++) {
    indices_[i] = i;
  }

  // 全幅数据构建树
  root_ = build_tree_for_range(0, data_num_ - 1, NULL);

  // std::cout << std::endl
  //           << "tree  " << data_num_ << "  " << num_of_nodes << std::endl;
}

KDTreeNode* KDTree::build_tree_for_range(int l /*lower bound*/,
                                         int u /*upper bound*/,
                                         KDTreeNode* parent) {
  if (u < l) {
    return (NULL);  // no data in this node.
  }

  // recursive function to build
  KDTreeNode* node;
  try {
    node = new KDTreeNode(dim_);

    // Debug
    // num_of_nodes++;
    // if (num_of_nodes % 1000 == 0) {
    //   int delay = 5;
    //   delay--;
    // }

  } catch (std::bad_alloc) {
    std::cout << "memory leak!" << std::endl;
  }

  // 构建叶子节点与非叶子节点
  // 控制树的深度，提高搜索效率
  if ((u - l) <= bucket_size) {  // create a terminal node.
    // always compute true bounding box for terminal node.
    // 计算叶子节点的每个维度上的范围
    for (int i = 0; i < dim_; i++) {
      spread_in_coordinate_box(i, l, u, node->box[i]);
    }
    // 终止划分，存储多个点
    node->lower   = l;
    node->upper   = u;
    node->cut_dim = 0;
    node->cut_val = 0.0;
    node->left = node->right = NULL;
  } else {
    // Compute an APPROXIMATE bounding box for this node.
    // if parent == NULL, then this is the root node, and
    // we compute for all dimensions.
    // Otherwise, we copy the bounding box from the parent for
    // all coordinates except for the parent's cut dimension.
    // That, we recompute ourself.
    //
    int c            = -1;  // for max spread
    float max_spread = 0.0;

    // 只有在 切分维度 上，子节点的边界框会发生变化，
    // 因此需要重新计算。其他 未切分的维度 可以直接从父节点继承边界框。
    for (int i = 0; i < dim_; i++) {
      if ((parent == NULL) || (parent->cut_dim == i)) {
        spread_in_coordinate_box(i, l, u, node->box[i]);
      } else {
        node->box[i] = parent->box[i];
      }
      // 选择扩展最大的维度
      float spread = node->box[i].upper - node->box[i].lower;
      if (spread > max_spread) {
        max_spread = spread;
        c          = i;
      }
    }

    // now, c is the identity of which coordinate has the greatest spread
    int mid_index = -1;
    if (split_method_ == SplitMethod::MEDIAN) {
      // 中位数切分
      mid_index = (l + u) / 2;
      select_on_coordinate_index(c, mid_index, l, u);
    } else if (split_method_ == SplitMethod::MEAN) {
      // 均值切分
      float sum = 0.0;
      for (int k = l; k <= u; k++) {
        sum += buff_[indices_[k] * dim_ + c];
      }
      float average = sum / static_cast<float>(u - l + 1);
      mid_index     = select_on_coordinate_value(c, average, l, u);
    } else if (split_method_ == SplitMethod::MIDPOINT) {
      // 使用边界中心切分
      float midpoint = (node->box[c].upper + node->box[c].lower) * 0.5;
      mid_index      = select_on_coordinate_value(c, midpoint, l, u);
    } else {
      std::cout << "Unknown split method" << std::endl;
    }

    // move the indices around to cut on dim 'c'.
    node->lower   = l;
    node->upper   = u;
    node->cut_dim = c;
    node->left    = build_tree_for_range(l, mid_index, node);
    node->right   = build_tree_for_range(mid_index + 1, u, node);

    if (node->right == NULL) {
      for (int i = 0; i < dim_; i++) {
        node->box[i] = node->left->box[i];
      }
      node->cut_val      = node->left->box[c].upper;
      node->cut_val_left = node->cut_val_right = node->cut_val;
    } else if (node->left == NULL) {
      for (int i = 0; i < dim_; i++) {
        node->box[i] = node->right->box[i];
      }
      node->cut_val      = node->right->box[c].upper;
      node->cut_val_left = node->cut_val_right = node->cut_val;
    } else {
      node->cut_val       = (node->cut_val_left + node->cut_val_right) / 2.0;
      node->cut_val_right = node->right->box[c].lower;
      node->cut_val_left  = node->left->box[c].upper;
      //
      // now recompute true bounding box as union of subtree boxes.
      // This is now faster having built the tree, being logarithmic in
      // N, not linear as would be from naive method.
      //
      for (int i = 0; i < dim_; i++) {
        node->box[i].upper =
            std::max(node->left->box[i].upper, node->right->box[i].upper);
        node->box[i].lower =
            std::min(node->left->box[i].lower, node->right->box[i].lower);
      }
    }
  }
  return (node);
}

void KDTree::spread_in_coordinate_box(int cut_dim, int l, int u,
                                      interval& interv) {
  // return the minimum and maximum of the indexed data between l and u in
  // smin_out and smax_out.

  float smin, smax;
  float lmin, lmax;

  smin = buff_[indices_[l] * dim_ + cut_dim];
  smax = smin;

  // process two at a time.
  for (int i = l + 2; i <= u; i += 2) {
    /* way 1
    lmin = buff_[indices_[i - 1] * dim_ + cut_dim];
    lmax = buff_[indices_[i] * dim_ + cut_dim];

    if (lmin > lmax) {
      swap(lmin, lmax);
    }

    if (smin > lmin) smin = lmin;
    if (smax < lmax) smax = lmax;
    */
    // way 2
    auto [lmin, lmax] = std::minmax(buff_[indices_[i - 1] * dim_ + cut_dim],
                                    buff_[indices_[i] * dim_ + cut_dim]);

    smin = std::min(smin, lmin);
    smax = std::max(smax, lmax);
  }

  // is there one more element?
  // 处理最后一个点（如果 `u - l + 1` 是奇数）
  if ((u - l + 1) % 2 != 0) {
    float last = buff_[indices_[u] * dim_ + cut_dim];
    // if (smin > last) smin = last;
    // if (smax < last) smax = last;
    smin = std::min(smin, lmin);
    smax = std::max(smax, lmax);
  }

  interv.lower = smin;
  interv.upper = smax;
}

// 按索引 k 分割，确保 k 位置的元素是整个 [l, u] 范围内的第 k 小值，左侧的都更小，右侧的都更大
void KDTree::select_on_coordinate_index(int c, int k, int l, int u) {
  //  Move indices in ind[l..u] so that the elements in [l .. k]
  //  are less than the [k+1..u] elmeents, viewed across dimension 'c'.
  //

  while (l < u) {
    int t = indices_[l];  // 选定当前范围的第一个元素作为“基准值”
    int m = l;  // 记录最终 pivot 位置

    // 遍历 [l+1, u]，将小于 pivot 的元素交换到前面
    for (int i = l + 1; i <= u; i++) {
      if (buff_[indices_[i] * dim_ + c] < buff_[t * dim_ + c]) {
        m++;
        std::swap(indices_[i], indices_[m]);
      }
    }

    // 交换 pivot 到正确位置
    std::swap(indices_[l], indices_[m]);

    if (m <= k) l = m + 1;
    if (m >= k) u = m - 1;
  }  // while loop
}

// 按值 alpha 分割，将数据划分为 ≤ alpha 和 > alpha 两部分
int KDTree::select_on_coordinate_value(int c, float alpha, int l, int u) {
  //  Move indices in ind[l..u] so that the elements in [l .. return]
  //  are <= alpha, and hence are less than the [return+1..u]
  //  elments, viewed across dimension 'c'.
  //
  // lb 指向左侧的最后一个有效位置
  // ub 指向右侧的第一个无效位置
  int lb = l, ub = u;

  while (lb < ub) {
    if (buff_[indices_[lb] * dim_ + c] <= alpha) {
      // 该点在左侧（小于等于 alpha），不需要交换
      lb++;  // good where it is.
    } else {
      // 该点在右侧（大于 alpha），交换到后面
      std::swap(indices_[lb], indices_[ub]);
      ub--;
    }
  }

  // modified by FH. 避免所有元素都小于或大于 alpha
  // 所有元素 <= alpha → lb == u
  // 所有元素 > alpha → lb == l
  if (lb == u || lb == l) {
    // 直接返回 中点索引，避免不合理划分
    // 间接防止 lb - 1 溢出
    return (l + int((u - l) / 2));
  }

  // here ub = lb
  // lb 最终的位置可能不完全符合 <= alpha，所以这里做一次最终检查：
  // 防止 lb - 1 溢出
  if (buff_[indices_[lb] * dim_ + c] <= alpha)
    return (lb);
  else
    return (lb - 1);
}

void KDTree::search(KDTreeNode* node, SearchRecord& sr) {
  // the core search routine.
  // This uses true distance to bounding box as the
  // criterion to search the secondary node.
  //
  // This results in somewhat fewer searches of the secondary nodes
  // than 'search', which uses the vdiff vector,  but as this
  // takes more computational time, the overall performance may not
  // be improved in actual run time.

  if (!node) return;

  if ((node->left == NULL) && (node->right == NULL)) {
    // we are on a terminal node
    if (sr.nn == 0) {
      process_terminal_node_fixedball(node, sr);
    } else {
      process_terminal_node_knn(node, sr);
    }
  } else {
    KDTreeNode *ncloser, *nfarther;

    float extra;
    float qval = sr.qv[node->cut_dim];
    // value of the wall boundary on the cut dimension.
    if (qval < node->cut_val) {
      ncloser  = node->left;
      nfarther = node->right;
      extra    = node->cut_val_right - qval;
    } else {
      ncloser  = node->right;
      nfarther = node->left;
      extra    = qval - node->cut_val_left;
    };

    if (ncloser != NULL) {
      this->search(ncloser, sr);
    }

    if ((nfarther != NULL) &&
        (apollo::common::math::Square(extra) < sr.ball_size)) {
      // first cut
      if (this->box_in_search_range(nfarther, sr)) {
        this->search(nfarther, sr);
      }
    }
  }
}

void KDTree::knn_search(std::vector<float>& qv, int nn,
                        KDTreeResultVector& result) {
  SearchRecord sr(qv, *this, result);
  // std::vector<float> vdiff(dim_, 0.0);
  result.data.clear();
  sr.center_idx  = -1;
  sr.index_range = 0;
  sr.nn          = nn;

  // root_->search(sr);
  this->search(this->root_, sr);

  if (sort_results_) {
    sort(result.data.begin(), result.data.end());
  }
}

void KDTree::knn_search_around_point(int idx_in, int index_range, int nn,
                                     KDTreeResultVector& result) {
  std::vector<float> qv(dim_);  //  query vector
  for (int i = 0; i < dim_; i++) {
    qv[i] = buff_[idx_in * dim_ + i];
  }
  // copy the query vector.

  {
    SearchRecord sr(qv, *this, result);
    result.data.clear();
    // construct the search record.
    sr.center_idx  = idx_in;
    sr.index_range = index_range;
    sr.nn          = nn;

    // root->search(sr);
    this->search(this->root_, sr);
  }

  if (sort_results_) {
    sort(result.data.begin(), result.data.end());
  }
}

void KDTree::knn_search_brute_force(std::vector<float>& qv, int nn,
                                    KDTreeResultVector& result) {
  // 该函数只做了暴力搜索，没有使用kd-tree，并且没有结果没有真正选择最近的 nn 个点
  result.data.clear();

  // 把所有点都拿来计算距离
  for (int i = 0; i < data_num_; i++) {
    float dis = 0.0;
    KDTreeResult e;
    for (int j = 0; j < dim_; j++) {
      dis += apollo::common::math::Square(buff_[i * dim_ + j] - qv[j]);
    }
    // 写入的是距离的平方
    e.dis = dis;
    e.idx = i;
    result.data.push_back(e);

    // 维护nn个最近点
    // some_func();
  }

  if (sort_results_) {
    sort(result.data.begin(), result.data.end());
  }
}

void KDTree::radius_search(std::vector<float>& qv, float r2,
                           KDTreeResultVector& result) {
  // search for all within a ball of a certain radius
  SearchRecord sr(qv, *this, result);
  // std::vector<float> vdiff(dim_, 0.0);
  result.data.clear();
  sr.center_idx  = -1;
  sr.index_range = 0;
  sr.ball_size   = r2;

  // root_->search(sr);
  this->search(this->root_, sr);

  if (sort_results_) {
    sort(result.data.begin(), result.data.end());
  }
}

void KDTree::radius_search_around_point(int idx_in, int index_range, float r2,
                                        KDTreeResultVector& result) {
  std::vector<float> qv(dim_);  //  query vector
  for (int i = 0; i < dim_; i++) {
    qv[i] = buff_[idx_in * dim_ + i];
  }
  // copy the query vector.

  {
    SearchRecord sr(qv, *this, result);
    result.data.clear();
    // construct the search record.
    sr.center_idx  = idx_in;
    sr.index_range = index_range;
    sr.ball_size   = r2;

    this->search(this->root_, sr);
  }

  if (sort_results_) {
    sort(result.data.begin(), result.data.end());
  }
}

int KDTree::r_count(std::vector<float>& qv, float r2) {
  // search for all within a ball of a certain radius
  KDTreeResultVector result;
  SearchRecord sr(qv, *this, result);
  sr.center_idx  = -1;
  sr.index_range = 0;
  sr.ball_size   = r2;

  this->search(this->root_, sr);

  return (result.data.size());
}

int KDTree::r_count_around_point(int idx_in, int index_range, float r2) {
  std::vector<float> qv(dim_);  //  query vector
  for (int i = 0; i < dim_; i++) {
    qv[i] = buff_[idx_in * dim_ + i];
  }
  // copy the query vector.

  {
    KDTreeResultVector result;
    SearchRecord sr(qv, *this, result);
    // construct the search record.
    sr.center_idx  = idx_in;
    sr.index_range = index_range;
    sr.ball_size   = r2;

    this->search(this->root_, sr);

    return (result.data.size());
  }
}

bool KDTree::box_in_search_range(KDTreeNode* node, SearchRecord& sr) {
  // does the bounding box, represented by minbox[*],maxbox[*]
  // have any point which is within 'sr.ballsize' to 'sr.qv'??

  int dim    = sr.dim;
  float dis2 = 0.0;
  for (int i = 0; i < dim; i++) {
    dis2 += apollo::common::math::Square(distance_from_boundary(
        sr.qv[i], node->box[i].lower, node->box[i].upper));
    if (dis2 > sr.ball_size) {
      return (false);
    }
  }
  return (true);
}

void KDTree::process_terminal_node_knn(KDTreeNode* node, SearchRecord& sr) {
  // knn
  int center_idx  = sr.center_idx;
  int index_range = sr.index_range;
  unsigned int nn = sr.nn;
  int dim         = sr.dim;
  float ball_size = sr.ball_size;
  float* data     = sr.buff_;

  const bool debug = false;
  if (debug) {
    printf("Processing terminal node %d, %d\n", node->lower, node->upper);
    std::cout << "Query vector = [";
    for (int i = 0; i < dim; i++) std::cout << sr.qv[i] << ',';
    std::cout << "]\n";
    std::cout << "nn = " << nn << '\n';
    // check_query_in_bound(sr);
  }

  for (int i = node->lower; i <= node->upper; i++) {
    int indexofi = sr.indices_[i];
    float dis    = 0.0;

    if (center_idx > 0) {
      // we are doing decorrelation interval
      if (abs(indexofi - center_idx) < index_range) {
        // skip this point.
        continue;
      }
    }

    bool early_exit = false;
    for (int k = 0; k < dim; k++) {
      dis += apollo::common::math::Square(data[indexofi * dim + k] - sr.qv[k]);
      if (dis > ball_size) {
        early_exit = true;
        break;
      }
    }
    if (early_exit) {
      // next iteration of mainloop
      continue;
    }

    // here the point must be added to the list.
    //
    // two choices for any point.  The list so far is either
    // undersized, or it is not.
    //
    if (sr.result.data.size() < nn) {
      KDTreeResult e;
      e.idx = indexofi;
      e.dis = dis;

      sr.result.push_element_and_heapify(e);
      if (sr.result.data.size() == nn) ball_size = sr.result.max_value();
      // Set the ball radius to the largest on the list (maximum priority).

      if (debug) {
        std::cout << "Unilaterally pushed dis=" << dis
                  << " ballsize = " << ball_size << "\n"
                  << "sr.result.size() = " << sr.result.data.size() << '\n';
      }
    } else {
      //
      // if we get here then the current node, has a squared
      // distance smaller
      // than the last on the list, and belongs on the list.
      //
      KDTreeResult e;
      e.idx = indexofi;
      e.dis = dis;

      ball_size = sr.result.replace_maxpri_elt_return_new_maxpri(e);

      if (debug) {
        std::cout << "Replaced maximum dis with dis=" << dis
                  << " new ballsize =" << ball_size << '\n';
      }
    }
  }  // main loop
  sr.ball_size = ball_size;
}

void KDTree::process_terminal_node_fixedball(KDTreeNode* node,
                                             SearchRecord& sr) {
  // 按半径搜索，固定半径 ==> ball_size
  int center_idx  = sr.center_idx;
  int index_range = sr.index_range;
  int dim         = sr.dim;
  float ball_size = sr.ball_size;
  float* data     = sr.buff_;

  for (int i = node->lower; i <= node->upper; i++) {
    // 构造函数中传入了 树的索引 sr.indices_ = this->indices_ ;
    int indexofi = sr.indices_[i];
    float dis    = 0.0;

    // 避免密集点/邻近点重复计算？
    if (center_idx > 0) {
      // we are doing decorrelation interval
      if (abs(indexofi - center_idx) < index_range) {
        // skip this point.
        continue;
      }
    }

    bool early_exit = false;
    for (int k = 0; k < dim; k++) {
      // 计算 索引数组点 和 查询点 的距离
      // 累加距离，到某一个维度时，超过阈值，则跳出循环
      dis += apollo::common::math::Square(data[indexofi * dim + k] - sr.qv[k]);
      if (dis > ball_size) {
        early_exit = true;
        break;
      }
    }
    // 跳出二重循环
    if (early_exit) {
      // next iteration of mainloop
      continue;
    }

    {
      KDTreeResult e;
      e.idx = indexofi;
      e.dis = dis;
      sr.result.data.push_back(e);
    }
  }
}

void KDTree::FreeTree() {
  if (root_ != NULL) {
    delete root_;
  }
}

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo
