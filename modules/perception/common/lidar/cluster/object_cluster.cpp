#include "modules/perception/common/lidar/cluster/object_cluster.h"

namespace jojo {
namespace perception {
namespace lidar {
namespace algorithm = jojo::perception::algorithm;
namespace base      = jojo::perception::base;

ObjectCluster::ObjectCluster() {}

ObjectCluster::~ObjectCluster() {
  if (max_data_3D != nullptr) {
    delete[] max_data_3D;
  }
  if (max_data_2D != nullptr) {
    delete[] max_data_2D;
  }
  if (data_intensity != nullptr) {
    delete[] data_intensity;
  }
  // ==> ~KDTree();
  // max_kdtree->FreeTree();
  if (max_kdtree != nullptr) {
    // delete max_kdtree;
    // 释放旧的指针 or 生命周期交给引用计数管理。
    // max_kdtree.reset();   
  }
}

void ObjectCluster::init3d(int width, int height, int length) {
  if (initialized_) return;
  initialized_ = true;

  // Max Lidar Point Number
  // at most 2100*64 < 135000 points
  map_size    = width * height * length;
  max_data_3D = new float[map_size * 3];
  // data_intensity = new unsigned char[map_size];
  data_intensity = new float[map_size];
  // max_kdtree = new algorithm::KDTree();
  max_kdtree = std::make_shared<algorithm::KDTree>();
  max_kdtree->init(max_data_3D, 3, map_size, true);

  data_type_ = 1;
}

// no use
void ObjectCluster::init2d(int width, int height) {
  if (initialized_) return;
  initialized_ = true;

  map_size    = width * height;
  max_data_2D = new float[map_size * 3];
  // max_kdtree = new algorithm::KDTree();
  max_kdtree = std::make_shared<algorithm::KDTree>();
  max_kdtree->init(max_data_2D, 2, map_size, true);

  data_type_ = 0;
}

void ObjectCluster::set_params(float eps, int minPts) {
  this->radius_ = eps;
  this->minPts_ = minPts;
}

void ObjectCluster::SetInputCloud(float* cloud, unsigned char* intensity,
                                  int number) {
  memset(max_data_3D, 0, sizeof(float) * map_size * 3);  // 清空整个内存
  memset(data_intensity, 0, sizeof(unsigned char) * map_size);

  memcpy(max_data_3D, cloud, sizeof(float) * number * 3);
  memcpy(data_intensity, intensity, sizeof(unsigned char) * number);

  data_num = number;
}

void ObjectCluster::SetInputCloud(float* cloud, float* intensity, int number) {
  memset(max_data_3D, 0, sizeof(float) * map_size * 3);
  memset(data_intensity, 0, sizeof(float) * map_size);

  memcpy(max_data_3D, cloud, sizeof(float) * number * 3);
  memcpy(data_intensity, intensity, sizeof(float) * number);

  data_num = number;
}

void ObjectCluster::SetInputCloud(float* cloud, int number) {
  // memcpy(max_data_2D, cloud, sizeof(float) * number * 2);
  for (int index = 0; index < number; index++) {
    max_data_2D[index * 2]     = cloud[index * 2];
    max_data_2D[index * 2 + 1] = cloud[index * 2 + 1];
  }

  data_num = number;
}

void ObjectCluster::SetInputCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr) {
  data_num = cloud_ptr->size();

  /* way 1
  for (int i = 0; i < data_num; ++i) {
    max_data_3D[i * 3]     = cloud_ptr->points[i].x;
    max_data_3D[i * 3 + 1] = cloud_ptr->points[i].y;
    max_data_3D[i * 3 + 2] = cloud_ptr->points[i].z;
    // data_intensity[i] =
    //     static_cast<unsigned char>(cloud_ptr->points[i].intensity);
    data_intensity[i] = cloud_ptr->points[i].intensity;
  }
  */
  // way 2 指针一次取出
  const auto* pts  = cloud_ptr->points.data();
  float* xyz_ptr   = max_data_3D;  // 存 n*3 float
  float* inten_ptr = data_intensity;  // 存 n float
  for (int i = 0; i < data_num; ++i) {
    const auto& p = pts[i];

    // XYZ 连续写入，避免多次计算索引
    *xyz_ptr++ = p.x;
    *xyz_ptr++ = p.y;
    *xyz_ptr++ = p.z;

    // intensity
    *inten_ptr++ = p.intensity;
  }
}

void ObjectCluster::SetInputCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
  data_num = cloud_ptr->size();

  /* way 1
  for (int i = 0; i < data_num; ++i) {
    max_data_3D[i * 3]     = cloud_ptr->points[i].x;
    max_data_3D[i * 3 + 1] = cloud_ptr->points[i].y;
    max_data_3D[i * 3 + 2] = cloud_ptr->points[i].z;
    data_intensity[i]      = 0.f;
  }
  */
  // way 2
  const auto* pts  = cloud_ptr->points.data();
  float* xyz_ptr   = max_data_3D;  // 存 n*3 float
  float* inten_ptr = data_intensity;  // 存 n float
  for (int i = 0; i < data_num; ++i) {
    const auto& p = pts[i];

    // XYZ 连续写入，避免多次计算索引
    *xyz_ptr++ = p.x;
    *xyz_ptr++ = p.y;
    *xyz_ptr++ = p.z;

    // intensity
    *inten_ptr++ = 0;
  }
}

void ObjectCluster::Run(int mode) {
  max_kdtree->build_tree(data_num);

  // -1: 噪声, 0: 未分类, 其他: 簇 ID
  this->labels_.assign(this->data_num, 0);
  this->cluster_id_ = 1;  // 从 1 开始
  this->ClusterVector.clear();  // Cluster Result
  this->OutputObjectVector.clear();

  std::vector<float> query(3);
  // float query[3];

  algorithm::KDTreeResultVector neighbors;
  // neighbors.data[0].idx ==> 邻域点的索引
  // 名字太长，赋值给 n_ind，方便使用
  // neighbors_index
  std::vector<int> n_ind;
  // 对每一个未分类的点，执行以下操作
  for (int index = 0; index < data_num; index++) {
    // 如果该点已经被分类，跳过
    if (labels_[index] != 0) {
      continue;
    }

    // ==> RegionQuery()
    // ######## RegionQuery Start ########
    // 未被分类，获取坐标，进行半径搜索
    switch (data_type_) {
      case 0: {
        query[0] = max_data_2D[index * 2];
        query[1] = max_data_2D[index * 2 + 1];
        query[2] = 0;
        break;
      }
      case 1: {
        query[0] = max_data_3D[index * 3];
        query[1] = max_data_3D[index * 3 + 1];
        query[2] = max_data_3D[index * 3 + 2];
        break;
      }
      default:
        std::cout << "data_type_ set error in Run func " << std::endl;
        break;
    }

    // 默认是坐标系原点距离
    // 根据距离动态调整半径
    float distance = this->distance(query);
    float radius   = this->GetRadius(distance);

    // 对每个点进行半径搜索
    max_kdtree->radius_search(query, radius, neighbors);
    // sr 对 neighbors 进行封装，获得对应点的索引
    // neighbors.data[index] = KDTreeResult

    // ######## RegionQuery End ########

    // 获得每个点的邻域内的点的索引
    n_ind.clear();
    for (size_t j = 0; j < neighbors.data.size(); j++) {
      n_ind.push_back(neighbors.data[j].idx);
    }

    // ==> ExpandCluster()
    // way 1
    if (mode == 1) {
      // this->OnceRegionGrowing(index, n_ind);
      this->RegionGrowing(index, n_ind);
      // this->MaxRegionGrowing(index, n_ind);
    }
    // way 2
    else if (mode == 2) {
      this->DBSCAN();
    } else {
      std::cout << " object cluster mode set error " << mode << std::endl;
    }
  }

  this->FilterClusters();
}

float ObjectCluster::GetRadius(float& distance) {
  float radius;

  // 变体 DBSCAN -> eps ; 区域生长 -> radius
  // way 1 人为设置近中远 三个距离
  /*
  // 距离车子较近时取小的radius
  if (distance < hps_.far) {  
    radius = radius_;
  } else if (distance < hps_.mid) {
    radius = hps_.far_r_sacle * radius_;
  } else {
    radius = hps_.mid_r_sacle * radius_;
  }
  */
  // way 2 根据距离线性增大聚类半径
  // 近处点密集，半径太大会合并过多目标；
  // 远处点稀疏，需要适当放大聚类半径；
  // 但太远时点云稀疏得几乎不成簇，固定一个最大值更稳妥。
  /*
  // 距离车子较远时取默认的radius
  if (distance > hps_.mid) {
    radius = radius_;
  } else {
    // by lbk 20210509
    radius = distance * hps_.r_sacle;  // ?
  }
  */
  radius = (distance > hps_.mid) ? radius_ : distance * hps_.r_sacle;

  //  return squared(radius);
  return radius;
}

// 区域生长，采用: 边聚类，边更新簇内点云
// 适用于小规模点云 或 实时点云处理。
void ObjectCluster::OnceRegionGrowing(int& ind, std::vector<int>& n_ind) {
  // 未采用嵌套的方式，而是使用大循环遍历
  // 区域大小 取决于邻域点的数量
  base::Segment tmp_segment;

  // ######## ExpandCluster Start ########
  // 初始化与普通的区域生长保持一致，为了生成最开始的区域
  // 当前点的类别值
  // int qv_c_id = labels_[ind];
  // 0 未分类，新增一类
  // if (qv_c_id == 0) {
  labels_[ind] = this->cluster_id_;

  this->UpdateSegment(ind, labels_[ind] /*this->cluster_id_*/, tmp_segment);

  for (size_t j = 0; j < n_ind.size(); j++) {
    if (n_ind[j] == ind) {  // 跳过自己
      continue;
    }

    // 如果邻域中存在已被其他类别标注的点：
    // 当前点就不加入这个区域 或者 不往该方向扩展。
    // 适用于：分割清晰、类别互斥的场景，比如语义分割、点云聚类等。
    if (labels_[n_ind[j]] != 0) {  // 跳过邻域内的已分类点
      continue;
    }
    labels_[n_ind[j]] = labels_[ind];

    this->UpdateSegment(n_ind[j], labels_[ind], tmp_segment);
  }

  ClusterVector.emplace_back(tmp_segment);
  this->cluster_id_++;
  // }
}

void ObjectCluster::RegionGrowing(int& ind, std::vector<int>& n_ind) {
  // 采用嵌套的方式递归
  base::Segment tmp_segment;

  std::queue<int> seeds;  // 生长队列

  // ######## ExpandCluster Start ########
  // 不是 0 未分类 无法进入到该函数
  // if (qv_c_id == 0) {
  labels_[ind] = this->cluster_id_;

  this->UpdateSegment(ind, labels_[ind] /*this->cluster_id_*/, tmp_segment);

  for (size_t j = 0; j < n_ind.size(); j++) {
    // 合并成一个代码，因为自身已被分类
    // if (n_ind[j] == ind) {  // 跳过自己
    //   continue;
    // }
    if (labels_[n_ind[j]] != 0) {  // 跳过邻域内的已分类点
      continue;
    }
    seeds.push(n_ind[j]);
  }

  while (!seeds.empty()) {
    int current = seeds.front();
    seeds.pop();

    std::vector<float> query(3);
    switch (data_type_) {
      const float* ptr;
      case 0:
        ptr      = &max_data_2D[current * 2];
        query[0] = ptr[0];
        query[1] = ptr[1];
        query[2] = 0.f;
        break;
      case 1:
        ptr      = &max_data_3D[current * 3];
        query[0] = ptr[0];
        query[1] = ptr[1];
        query[2] = ptr[2];
        break;
      default:
        std::cout << "data_type error in RegionGrowing func " << std::endl;
        break;
    }

    float distance = this->distance(query);
    float radius   = GetRadius(distance);

    algorithm::KDTreeResultVector neighbors;
    max_kdtree->radius_search(query, radius, neighbors);

    for (auto& result : neighbors.data) {
      int nid = result.idx;

      // 合并成一个代码，因为自身已被分类
      if (labels_[nid] != 0) {
        continue;  // 已归类
      }

      labels_[nid] = cluster_id_;  // 归入当前簇
      seeds.push(nid);  // 加入扩展队列
      this->UpdateSegment(nid, cluster_id_, tmp_segment);
    }
  }

  this->ClusterVector.emplace_back(tmp_segment);
  this->cluster_id_++;
}

void ObjectCluster::MaxRegionGrowing(int& ind, std::vector<int>& n_ind) {
  base::Segment tmp_segment;

  std::queue<int> seeds;  // 生长队列

  // ######## ExpandCluster Start ########
  int cur_cluster_id = this->cluster_id_;
  labels_[ind]       = this->cluster_id_;

  this->UpdateSegment(ind, labels_[ind] /*this->cluster_id_*/, tmp_segment);

  for (size_t j = 0; j < n_ind.size(); j++) {
    if (labels_[n_ind[j]] != 0) {  // 跳过邻域内的已分类点
      continue;
    }
    seeds.push(n_ind[j]);
  }

  while (!seeds.empty()) {
    int current = seeds.front();
    seeds.pop();

    std::vector<float> query(3);
    switch (data_type_) {
      case 0:
        query[0] = max_data_2D[current * 2];
        query[1] = max_data_2D[current * 2 + 1];
        query[2] = 0;
        break;
      case 1:
        query[0] = max_data_3D[current * 3];
        query[1] = max_data_3D[current * 3 + 1];
        query[2] = max_data_3D[current * 3 + 2];
        break;
      default:
        std::cout << "data_type error in MaxRegionGrowing func " << std::endl;
        break;
    }

    float distance = this->distance(query);
    float radius   = GetRadius(distance);

    algorithm::KDTreeResultVector neighbors;
    max_kdtree->radius_search(query, radius, neighbors);

    for (auto& result : neighbors.data) {
      int nid = result.idx;

      if (nid == current) {
        continue;
      }

      if (labels_[nid] == 0) {
        labels_[nid] = cur_cluster_id;
        seeds.push(nid);
        this->UpdateSegment(nid, cur_cluster_id, tmp_segment);
      } else if (labels_[nid] != cur_cluster_id) {
        // 已归属于其他簇：进行合并判断
        int other_id = labels_[nid];
        this->MergeClusters(cur_cluster_id, other_id);
      }
    }
  }

  this->cluster_id_++;
}

void ObjectCluster::MergeClusters(int id_a, int id_b) {
  if (id_a == id_b) {
    return;
  }

  base::Segment* seg_a = nullptr;
  base::Segment* seg_b = nullptr;

  // 找到两个目标 Segment 指针
  for (auto& seg : ClusterVector) {
    if (seg.id == -1) {
      continue;  // 已经被清空的簇
    }

    if (seg.id == id_a) {
      seg_a = &seg;
    }
    if (seg.id == id_b) {
      seg_b = &seg;
    }
  }

  // 未找到对应簇
  if (!seg_a || !seg_b) {
    return;
  }

  base::Segment* keep_seg =
      (seg_a->points->size() >= seg_b->points->size()) ? seg_a : seg_b;
  base::Segment* drop_seg = (keep_seg == seg_a) ? seg_b : seg_a;

  int keep_id = keep_seg->id;
  int drop_id = drop_seg->id;

  // 合并点
  // clang-format off
  keep_seg->points->insert(keep_seg->points->end(), 
                           drop_seg->points->begin(),
                           drop_seg->points->end());

  keep_seg->points_index_vector.insert(keep_seg->points_index_vector.end(),
                                       drop_seg->points_index_vector.begin(),
                                       drop_seg->points_index_vector.end());
  // clang-format on

  // 更新标签
  for (int i = 0; i < drop_seg->points_index_vector.size(); ++i) {
    int drop_idx = drop_seg->points_index_vector[i];
    if (labels_[drop_idx] == drop_id) {
      labels_[drop_idx] = keep_id;
    }
  }

  // 清空丢弃簇的数据
  drop_seg->points->clear();
  drop_seg->points_index_vector.clear();
  drop_seg->id = -1;  // 标记为无效，可在后续清理

  // 为什么没有真的去清理，因为清理需要动态更新 this->cluster_id_ = new_id;
}

void ObjectCluster::UpdateSegment(int& ind, int& c_id, base::Segment& segment) {
  if (segment.id == -1) {
    segment.id = c_id;
  }

  /* expand
  PointF pt;
  unsigned char tmp_intensity;

  pt.x = max_data_3D[ind * 3];
  pt.y = max_data_3D[ind * 3 + 1];
  pt.z = max_data_3D[ind * 3 + 2];

  pt.intensity = data_intensity[index];
  */
  pcl::PointXYZI pt;
  const float* p = &max_data_3D[ind * 3];  // 一次寻址，减少乘法

  pt.x = p[0];
  pt.y = p[1];
  pt.z = p[2];

  pt.intensity = data_intensity[ind];

  // 直接 emplace_back，跳过拷贝
  segment.points->emplace_back(pt);
  // 保存索引
  segment.points_index_vector.push_back(ind);
}

void ObjectCluster::FilterClusters() {
  for (auto& segment : this->ClusterVector) {
    if (!segment.points->empty()) {
      int tmp_nMin;

      // 粗筛选
      float distance =
          std::sqrt(segment.points->points[0].x * segment.points->points[0].x +
                    segment.points->points[0].y * segment.points->points[0].y);

      // 此处单位为米，基于雷达的探测距离100米设置
      if (distance > hps_.far) {
        tmp_nMin = this->minPts_ * hps_.far_e_scale;
      } else if (distance > hps_.mid) {
        tmp_nMin = this->minPts_ * hps_.mid_e_scale;
      } else {
        tmp_nMin = this->minPts_;
      }

      if (segment.points->size() >= tmp_nMin) {
        // 方式 1：直接用 make_shared 并 move 构造
        auto segment_ptr = std::make_shared<base::Segment>(std::move(segment));
        algorithm::CalculateSegAttribute(segment_ptr);
        this->OutputObjectVector.push_back(segment_ptr);
      }
    }
  }
}

// DBSCAN，采用: 先分配聚类标签，再统一更新点云
// 适用于大规模点云处理
void ObjectCluster::DBSCAN() {}

float ObjectCluster::distance(const std::vector<float>& a) {
  // 开方耗时，不开方能行吗
  /* way 1
  float spatial_distance;
  switch (data_type_) {
    case 0:
      spatial_distance = std::sqrt(a[0] * a[0] + a[1] * a[1]);
      break;
    case 1:
      spatial_distance = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      break;
    default:
      std::cout << "data_type_ set error in disntance func " << std::endl;
      break;
  }
  */
  float spatial_distance = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  return spatial_distance;
}

bool ObjectCluster::GetOutputSegs(
    std::vector<std::shared_ptr<base::Segment>>& SegVector) {
  SegVector.clear();

  // way 1 手动执行内部数据的深拷贝
  /*
  for (const auto &seg : OutputObjectVector) {
    base::Segment copy_seg = seg;
    if (seg.points) {
      copy_seg.points =
          boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*seg.points);
    }
    SegVector.emplace_back(copy_seg);
  }
  */

  // way 2 struct 内部实现了 = 运算符重载，可直接深拷贝
  // way 3 修改为智能指针，直接赋值即可
  SegVector = OutputObjectVector;

  return true;
}

void ObjectCluster::ClusterPolicy(std::shared_ptr<base::Segment>& result,
                                  uint mode) {
  switch (mode) {
    case 1:
      algorithm::GetLargestCluster(this->OutputObjectVector, result);
      break;
    case 2:
      algorithm::GetClosestAmongTop2Clusters(this->OutputObjectVector, result);
      break;
    default:
      break;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace jojo