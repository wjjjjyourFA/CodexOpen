#include "modules/localization/fast_lio/map_localization.h"

namespace fastlio {

MapLocalization::MapLocalization() : LidarOdometry() {
  // feats_undistort.reset(new PointCloudXYZI());
  // feats_down_body.reset(new PointCloudXYZI());
  // feats_down_world.reset(new PointCloudXYZI());
  // normvec.reset(new PointCloudXYZI(100000, 1));
  // laserCloudOri.reset(new PointCloudXYZI(100000, 1));
  // corr_normvect.reset(new PointCloudXYZI(100000, 1));
  // p_imu = std::make_shared<ImuProcess>();

  // feats_undistort_filtered.reset(new PointCloudXYZI());
}

MapLocalization::~MapLocalization() {}

void MapLocalization::Init(
    std::shared_ptr<jojo::localization::RuntimeConfig> rparam,
    std::shared_ptr<jojo::localization::StaticConfig> sparam) {
  param_  = rparam;
  sparam_ = sparam;

  // 调用父类的初始化函数
  LidarOdometry::Init(param_);
  // “static 回调 + virtual 分发” ==> 最终会调用 MapLocalization::h_share_model

  // for IMU_processing
  this->LoadInitMap(sparam_->map_file_path);

  this->SetMapCenter(sparam_->map_center);
}

void MapLocalization::LoadInitMap(const std::string& map_path) {
  // “变量命名：数据类型在前，作用/用途在后
  // std::cout << "map_file_path: " << map_path << std::endl;

  // 一般构建的高精地图是这个格式
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  double load_start = omp_get_wtime();
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *raw_cloud) == -1) {
    LOG(ERROR) << "Couldn't read map file from " << map_path;
    abort();
  }
  double load_end = omp_get_wtime();

  LOG(INFO) << "Loaded map " << raw_cloud->points.size()
            << " points from " + map_path;
  LOG(INFO) << "Load map cost: " << (load_end - load_start) * 1000 << "ms";

  // 为什么要转换格式？
  // 原 fast-lio 使用的是 pcl::PointXYZINormal
  PointCloudXYZI::Ptr cloud_map(new PointCloudXYZI);
  size_t map_size = raw_cloud->size();
  cloud_map->points.resize(map_size);

  for (size_t i = 0; i < map_size; ++i) {
    const auto& src = raw_cloud->points[i];
    PointType& dst  = cloud_map->points[i];

    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;

    dst.intensity = src.intensity;

    // other info
  }
  cloud_map->width    = raw_cloud->width;
  cloud_map->height   = raw_cloud->height;
  cloud_map->is_dense = raw_cloud->is_dense;

  this->SetInitMap(cloud_map);
}

void MapLocalization::SetInitMap(const PointCloudXYZI::Ptr& map) {
  // 指针持有初始地图，避免地图生命周期内释放
  // 深拷贝（deep copy）
  // this->map_.reset(new PointCloudXYZI);
  // this->*map_ = *map;
  // 浅拷贝（shared_ptr 共享）
  this->map_ = map;
  std::cout << "map size: " << map->points.size() << std::endl;

  // 构建 kdtree
  double build_start = omp_get_wtime();
  if (ikdtree.Root_Node == nullptr) {
    // 直接用 map 构建，不依赖 feats_down_size
    ikdtree.set_downsample_param(filter_size_map_min);
    // 根据世界坐标系下的点构建ikdtree
    ikdtree.Build(this->map_->points);
    // ikdtree_dyn 在出图的时候，依据 当时的 ikdtree 邻域构建
    LOG(INFO) << "ikdtree build done, tree size: " << ikdtree.size()
              << " validd num: " << ikdtree.validnum();
  }
  double build_end = omp_get_wtime();

  LOG(INFO) << "Build map cost: " << (build_end - build_start) * 1000 << "ms";

  assert(ikdtree.Root_Node != nullptr);
}

void MapLocalization::SetMapCenter(const Eigen::Vector3d& center) {
  map_center = center;
}

void MapLocalization::SetInitPose(const Eigen::Vector3d& pos,
                                  const Eigen::Quaterniond& rot) {
  // 用于外部设置初值，如 GNSS，此时不需要更新 kf
  init_state = kf.get_x();

  init_state.pos = pos;
  init_state.rot = rot;
}

void MapLocalization::UpdateKfState(state_ikfom& state_point) {
  // 参考 IMU_processing IMU_init()，初始化 kf
  // state_ikfom init_state 和 state_point 用途是不一样的;

  // TODO：输入一个 pose 直接更新 kf，这样做合理吗？
  // 不需要添加 state_point 的其他属性吗？
  // ==> 执行在 IMU_init() 之前，IMU_init() 中会更新 state_point 的其他属性
  kf.change_x(state_point);
}

void MapLocalization::pointBodyToWorld(PointType const* const pi,
                                       PointType* const po, state_ikfom& s) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  // no use
  po->intensity = pi->intensity;
}

void MapLocalization::NDT(pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr map_imu) {
  // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt;
  // ndt.reset(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>);
  auto ndt = boost::make_shared<
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();

  // 为终止条件设置最小转换差异
  ndt->setTransformationEpsilon(0.00001);
  // 为 More-Thuente 线搜索设置最大步长
  ndt->setStepSize(0.1);
  // 设置 NDT 网格结构的分辨率（VoxelGridCovariance）
  // 场景	       resolution
  // 室内         0.5 ~ 1.0
  // 城市道路     1.0 ~ 2.0
  // 高速 / 稀疏  2.0 ~ 5.0
  ndt->setResolution(1.5);
  // 设置匹配迭代的最大次数
  // 工程经验 30 ~ 80
  ndt->setMaximumIterations(50);
  ndt->setInputSource(frame_imu);
  ndt->setInputTarget(map_imu);

  pcl::PointCloud<pcl::PointXYZ>::Ptr o(new pcl::PointCloud<pcl::PointXYZ>);
  // 没有初始位姿
  // ndt->align(*o);
  // 使用初始位姿
  ndt->align(*o, this->Tr_delta);

  if (!ndt->hasConverged()) {
    std::cout << "NDT failed!" << std::endl;
    return;
  }

  double score = ndt->getFitnessScore();
  std::cout << "ndt score: " << score << std::endl;

  this->Tr_delta = ndt->getFinalTransformation();
}

void MapLocalization::ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr map_imu) {
  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp;
  // icp.reset(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
  auto icp = boost::make_shared<
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();

  // 设置收敛条件下的最小变换差
  icp->setTransformationEpsilon(0.00001);
  // 设置最大对应点对距离
  // 分阶段 ICP
  // 1.0 → 0.5 → 0.2
  icp->setMaxCorrespondenceDistance(1.0);
  icp->setMaximumIterations(100);  // 设置最大迭代次数
  icp->setInputSource(frame_imu);  // 设置输入点云（源）
  icp->setInputTarget(map_imu);  // 设置目标点云

  pcl::PointCloud<pcl::PointXYZ>::Ptr o(new pcl::PointCloud<pcl::PointXYZ>());
  icp->align(*o, this->Tr_delta);

  if (!icp->hasConverged()) {
    std::cout << "ICP failed!" << std::endl;
    return;
  }

  double score = icp->getFitnessScore();
  std::cout << "icp score: " << score << std::endl;

  this->Tr_delta = icp->getFinalTransformation();
}

void MapLocalization::GetAutoInitPose(state_ikfom& init_state,
                                      PointCloudXYZI::Ptr frame,
                                      PointCloudXYZI::Ptr map) {
  // !! map 是 world 坐标系下的点云; frame 是 lidar 坐标系下的点云;
  // 1. frame：Lidar → IMU
  // 2. map：world → IMU
  // 但这个变换过程是带误差的
  std::cout << "start auto init pose" << std::setprecision(6) << std::endl;

  Eigen::Affine3d Aff_imu_lidar = Eigen::Affine3d::Identity();
  // way 1 通过 IMU_PROCESSING 获取提前设置好的 lidar 到 IMU 的外参数
  Aff_imu_lidar.rotate(p_imu->Lidar_R_wrt_IMU);
  Aff_imu_lidar.translation() = p_imu->Lidar_T_wrt_IMU;
  // way 2 通过 状态估计获取 lidar 到 IMU 的外参数
  // Aff_imu_lidar.rotate(state_point.offset_R_L_I.matrix());
  // Aff_imu_lidar.translation() = state_point.offset_T_L_I;

  // Eigen::Matrix4d Tr_imu_lidar = Aff_imu_lidar.matrix();
  Eigen::Affine3f Aff_imu_lidar_f = Aff_imu_lidar.cast<float>();

  // IMU 在 world 坐标系下的位姿
  Eigen::Affine3d Aff_world_imu = Eigen::Affine3d::Identity();
  // 估计的 IMU 位姿 ==> init_state
  Aff_world_imu.rotate(init_state.rot.matrix());
  Aff_world_imu.translation() = init_state.pos;
  // std::cout << "Aff_world_imu: \n" << Aff_world_imu.matrix() << std::endl;

  Eigen::Affine3d Aff_imu_world = Aff_world_imu.inverse();

  // Eigen::Matrix4d Tr_imu_world = Aff_imu_world.matrix();
  Eigen::Affine3f Aff_imu_world_f = Aff_imu_world.cast<float>();

  // std::cout << "Aff_imu_lidar_f: \n" << Aff_imu_lidar_f.matrix() << std::endl;
  // std::cout << "Aff_imu_world_f: \n" << Aff_imu_world_f.matrix() << std::endl;

  // lidar 在 world 坐标系下的位姿
  // Eigen::Affine3f T_world_lidar = Aff_world_imu.cast<float>() * Aff_imu_lidar_f;

  // clang-format off
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu_ori(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_imu_ori(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_imu(new pcl::PointCloud<pcl::PointXYZ>);
  // clang-format on

  static float range_thresh_pow = 80 * 80;
  pcl::PointXYZ p_in, p_out;

  frame_imu_ori->points.reserve(frame->size());  // 避免反复扩容
  // const Eigen::Matrix3f R = Aff_imu_lidar_f.rotation();
  // const Eigen::Vector3f T = Aff_imu_lidar_f.translation();

  for (auto& pt : *frame) {
    /* way 1
    p_in.x = pt.x;
    p_in.y = pt.y;
    p_in.z = pt.z;
    // pcl::transformPoint 只支持 pcl::PointXYZ
    p_out = pcl::transformPoint(p_in, Aff_imu_lidar_f);

    float d2 = p_out.x * p_out.x + p_out.y * p_out.y;
    // float d2 = pow(p_out.x, 2) + pow(p_out.y, 2);
    */

    // /* way 2
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    // 直接手写变换（避免构造对象）
    Eigen::Vector3f p_out = Aff_imu_lidar_f * p;  // lidar -> imu
    // （比 Aff * p 更快）
    // Eigen::Vector3f p_out = R * p + T;

    float d2 = p_out.x() * p_out.x() + p_out.y() * p_out.y();
    // */

    // 用于初始位姿估计的点云输入
    // 依赖：近距离结构（地面、墙体）
    // 避免过滤掉：高处结构（桥、天花板）
    if (d2 < range_thresh_pow) {
      // frame_imu_ori->push_back(p_out);
      frame_imu_ori->emplace_back(p_out.x(), p_out.y(), p_out.z());
    }
  }

  map_imu_ori->points.reserve(map->size());

  for (auto& pt : *map) {
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    Eigen::Vector3f p_out = Aff_imu_world_f * p;  // world -> imu

    float d2 = p_out.x() * p_out.x() + p_out.y() * p_out.y();

    if (d2 < range_thresh_pow) {
      map_imu_ori->emplace_back(p_out.x(), p_out.y(), p_out.z());
    }
  }

  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(0.5, 0.5, 0.5);
  downSizeFilterSurf.setInputCloud(frame_imu_ori);
  downSizeFilterSurf.filter(*frame_imu);

  // downSizeFilterMap.setInputCloud(map_imu_ori);
  // downSizeFilterMap.filter(*map_imu);
  map_imu = map_imu_ori;

  std::cout << "frame size: " << frame_imu_ori->points.size() << std::endl;
  std::cout << "map size: " << map_imu_ori->points.size() << std::endl;

  if (sparam_->b_display_init) {
    ShowInitResult(vis, frame_imu, map_imu, "gnss_result");
  }

  std::cout << "start matching ..." << std::endl;

  // 设置匹配的初值
  this->Tr_delta             = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f Tr_delta_1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f Tr_delta_2 = Eigen::Matrix4f::Identity();

  this->NDT(frame_imu, map_imu);
  Tr_delta_1 = this->Tr_delta;
  std::cout << ".. end first matching ." << std::endl;

  if (sparam_->b_init_twice) {
    this->ICP(frame_imu, map_imu);
    Tr_delta_2 = this->Tr_delta;
    std::cout << ".. end second matching ." << std::endl;
  }

  Eigen::Affine3d Aff_delta = Eigen::Affine3d(this->Tr_delta.cast<double>());
  // std::cout << "Tr delta: \n " << Tr_delta << std::endl;

  // std::cout << "Tr before: \n " << Aff_world_imu.matrix() << std::endl;
  Aff_world_imu = Aff_world_imu * Aff_delta;
  std::cout << "After match init_state: \n " << Aff_world_imu.matrix() << endl;

  // 更新 初始位姿
  init_state.rot = Aff_world_imu.rotation();
  init_state.pos = Aff_world_imu.translation();

  if (sparam_->b_display_init) {
    // clang-format off
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu_corrected1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*frame_imu, *frame_imu_corrected1, Tr_delta_1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu_corrected2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*frame_imu, *frame_imu_corrected2, Tr_delta_2);

    // ShowMatchResultSingle(vis, frame_imu_corrected1, map_imu, "NDT");
    // ShowMatchResultSingle(vis, frame_imu_corrected2, map_imu, "ICP");
    ShowMatchResultDual(vis, frame_imu, frame_imu_corrected2, map_imu, "NDT ICP");
    ShowMatchResultDual(vis, frame_imu_corrected1, frame_imu_corrected2, map_imu, "RAW ICP" );
    // clang-format on
  }
}

void MapLocalization::InitDynMap() {
  // 构建一个新的 kdtree_dyn 用于存储 outside 地图 == 局部地图
  // 使用 出图的时刻的邻域 和 当前帧 构建 kdtree_dyn
  // 因为是共用的 state_point，因此地图范围外的更新地图，位姿是统一的，可以拼接；

  // TODO：是每次都重建好，还是利用滑窗自动删除超范围的点？
  // way 1 ikdtree_dyn.delete_tree_nodes();

  ikdtree_dyn.set_downsample_param(filter_size_map_min);

  // 1. 取 global 邻域
  PointType center;
  center.x = state_point.pos(0);
  center.y = state_point.pos(1);
  center.z = state_point.pos(2);

  // 手动获取，已经被历史 downsample 过
  PointVector local_pts;
  // 偏 地图构建 / 局部子图提取
  ikdtree.Radius_Search(center, dyn_map_radius_, local_pts);
  // ==> Nearest_Points 每个点都一组近邻点

  // 2. 当前帧转 world
  /* 是 scan-level downsample（不是 map-level）
  if (feats_down_size > 5) {
    feats_down_world->points.resize(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
      pointBodyToWorld(&(feats_down_body->points[i]),
                       &(feats_down_world->points[i]));
    }
  }
  */

  /* way 1 全局平衡树；KDTree结构 optimal（查询更快）
  // 一次拷贝
  // “去重策略不一致”
  PointVector fused = local_pts;
  fused.insert(fused.end(), feats_down_world->points.begin(),
               feats_down_world->points.end());
  ikdtree_dyn.Build(fused);
  */

  // way 2 Build 阶段的点已经做过 voxel / 最近邻约束，
  // 而 Add_Points 没有做，通过 Add_Points 触发
  ikdtree_dyn.Build(local_pts);
  // !! 在用两套“不同空间分布规则”的点，混进同一个树
  // ikdtree_dyn.Add_Points(feats_down_world->points, true);

  // 严格来说是，无法复用 map_incremental()，Nearest_Points 在 h_share_model() 后才有效
  // 但该流程有个硬性假设，一开始必在地图中，因此可认为 Nearest_Points 是有值的
  // 在同一帧中，map_incremental() 只执行一次，因此这里不执行叠加，交给后面叠加
  // this->map_incremental();

  LOG(INFO) << "Init dyn map";
}

void MapLocalization::lasermap_fov_segment() {
  cub_needrm.clear();  // 清空需要移除的区域
  kdtree_delete_counter = 0;
  kdtree_delete_time    = 0.0;

  // std::cout << "log message 1 : " << std::fixed << std::setprecision(4)
  //           << cube_len << " " << MOV_THRESHOLD << " " << DET_RANGE << " "
  //           << std::endl;

  // pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);

  // W系下位置
  V3D pos_LiD = pos_lid;
  // 初始化局部地图范围，以pos_LiD为中心,长宽高均为cube_len
  if (!Localmap_Initialized) {
    for (int i = 0; i < 3; i++) {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }

  // 各个方向上pos_LiD与局部地图边界的距离
  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    // 与某个方向上的边界距离（1.5*300m）太小，标记需要移除need_move(FAST-LIO2论文Fig.3)
    // 距离滑窗：通过移动距离判断局部地图的是否需要更新
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      need_move = true;
  }
  if (!need_move) return;  // 如果不需要，直接返回，不更改局部地图

  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  // 需要移动的距离
  float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                       double(DET_RANGE * (MOV_THRESHOLD - 1)));
  for (int i = 0; i < 3; i++) {
    tmp_boxpoints = LocalMap_Points;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      // 标记需要删除的区域
      cub_needrm.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  LocalMap_Points = New_LocalMap_Points;

  // 拿到“被删掉的点”，用于可视化；
  // points_cache_collect();

  // double delete_begin = omp_get_wtime();
  // 从 KD-tree 中删除 远离当前位姿的点
  if (cub_needrm.size() > 0) {
    // 只有已经构建了 ikdtree_dyn 才会执行该函数
    kdtree_delete_counter = ikdtree_dyn.Delete_Point_Boxes(cub_needrm);
  }
  // kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void MapLocalization::map_incremental() {
  // 增量更新地图
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);

  for (int i = 0; i < feats_down_size; i++) {
    /* transform to world frame 转换到世界坐标系 */
    pointBodyToWorld(&(feats_down_body->points[i]),
                     &(feats_down_world->points[i]));

    /* decide if need add to map */
    // if (!Nearest_Points[i].empty())
    if (!Nearest_Points[i].empty() && flg_EKF_inited) {
      const PointVector& points_near = Nearest_Points[i];
      bool need_add                  = true;
      BoxPointType Box_of_Point;
      // 点所在体素的中心
      PointType downsample_result, mid_point;
      mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      float dist  = calc_dist(feats_down_world->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
        // 如果距离最近的点都在体素外，则该点不需要Downsample
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        // 如果近邻点距离 < 当前点距离，不添加该点
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
    } else {
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  // double st_time = omp_get_wtime();
  add_point_size = ikdtree_dyn.Add_Points(PointToAdd, true);
  add_point_size += ikdtree_dyn.Add_Points(PointNoNeedDownsample, false);
  // ikdtree_dyn.Add_Points(PointNoNeedDownsample, false);
  // add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
  // kdtree_incremental_time = omp_get_wtime() - st_time;

  // std::cout << "add num: " << PointToAdd.size() << "  "
  //           << PointNoNeedDownsample.size() << " " << add_point_size
  //           << std::endl;
  // std::cout << "ikdtree_dyn num: " << ikdtree_dyn.size() << " " << ikdtree_dyn.validnum()
  //           << std::endl;
}

void MapLocalization::h_share_model(
    state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
  // 构建 观测模型（measurement model），用于计算残差 ==> ICP 的残差 + Jacobian
  // 点到地图的几何约束：点云（当前帧） → 对齐 → 地图
  // 最小化：点到平面的距离

  // std::cout << "MapLocalization::h_share_model" << std::endl;

  KD_TREE<PointType>* tree;

  if (enable_incremental_mapping_) {
    tree = &ikdtree_dyn;
  } else {
    tree = &ikdtree;
  }

  // 通过 ekfom_data 获取数据
  auto& feats_down_body  = ekfom_data.feats_down_body;
  auto& feats_down_world = ekfom_data.feats_down_world;
  auto& normvec          = ekfom_data.normvec;
  auto& laserCloudOri    = ekfom_data.laserCloudOri;
  auto& corr_normvect    = ekfom_data.corr_normvect;

  auto& Nearest_Points = ekfom_data.Nearest_Points;

  float* res_last           = ekfom_data.res_last;
  bool* point_selected_surf = ekfom_data.point_selected_surf;

  int feats_down_size = ekfom_data.feats_down_size;

  //
  double match_start = omp_get_wtime();
  laserCloudOri->clear();  // 计算点-面残差时，实际用到的满足要求的点坐标，l系
  corr_normvect->clear();  // 计算点-面残差时，实际用到的点对应平面的参数，w系
  total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
  // std::cout << "MP_EN ON" << std::endl;
#pragma omp parallel
  {
    std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

#pragma omp for
    // 遍历所有的特征点
    for (int i = 0; i < feats_down_size; i++) {
      PointType& point_body  = feats_down_body->points[i];
      PointType& point_world = feats_down_world->points[i];

      /* transform to world frame */
      V3D p_body(point_body.x, point_body.y, point_body.z);
      V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
      point_world.x         = p_global(0);
      point_world.y         = p_global(1);
      point_world.z         = p_global(2);
      point_world.intensity = point_body.intensity;

      // way 1 构造时分配
      // std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
      // way 2 resize 时分配
      // std::vector<float> pointSearchSqDis;
      // pointSearchSqDis.resize(NUM_MATCH_POINTS);
      // way 3 栈内存（最快）NUM_MATCH_POINTS  不能是运行时变量
      // float pointSearchSqDis[NUM_MATCH_POINTS];

      // auto& points_near = Nearest_Points[i];
      // zero copy
      auto& points_near = (*Nearest_Points)[i];
      points_near.clear();

      if (ekfom_data.converge) {
        /** Find the closest surfaces in the map **/
        // 偏 优化约束
        tree->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
                             pointSearchSqDis);
        point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false
                                 : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5
                                     ? false
                                     : true;
      }

      if (!point_selected_surf[i]) continue;

      VF(4) pabcd;
      point_selected_surf[i] = false;
      // 拟合局部平面
      if (esti_plane(pabcd, points_near, 0.1f)) {
        // 计算点到平面的距离（残差）
        // pd2 = a * x + b * y + c * z + d
        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                    pabcd(2) * point_world.z + pabcd(3);
        float s   = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

        // 过滤有效点
        if (s > 0.9) {
          point_selected_surf[i]       = true;
          normvec->points[i].x         = pabcd(0);
          normvec->points[i].y         = pabcd(1);
          normvec->points[i].z         = pabcd(2);
          normvec->points[i].intensity = pd2;
          res_last[i]                  = abs(pd2);
        }
      }
    }
  }
#else
  std::cout << "MP_EN is OFF" << std::endl;
  std::cerr << "Map Localization must enable MP_EN for performance 100ms !!"
            << std::endl;
  abort();
#endif

  effct_feat_num = 0;

  for (int i = 0; i < feats_down_size; i++) {
    if (point_selected_surf[i]) {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[i];
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    std::cerr << "No Effective Points! \n" << std::endl;
    return;
  }

  res_mean_last = total_residual / effct_feat_num;
  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
  ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  //23
  ekfom_data.h.resize(effct_feat_num);

  // 遍历被选中的特征点，计算测量值和测量雅可比矩阵
  for (int i = 0; i < effct_feat_num; i++) {
    const PointType& laser_p = laserCloudOri->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType& norm_p = corr_normvect->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (extrinsic_est_en) {
      // s.rot.conjugate()*norm_vec);
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    /*** Measuremnt: distance to the closest surface/corner ***/
    // 残差
    ekfom_data.h(i) = -norm_p.intensity;
  }
  solve_time += omp_get_wtime() - solve_start_;

  // LOG(INFO) << "effective points: " << effct_feat_num << std::endl;
}

void MapLocalization::run_localization(MeasureGroup& Measures) {
  // TODO：如果进来就超出地图范围怎么办？
  // !! 这里默认初始进来的时候，是在地图里的。

  if (flg_first_scan) {
    first_lidar_time        = Measures.lidar_beg_time;
    p_imu->first_lidar_time = first_lidar_time;
    flg_first_scan          = false;

    printf("Before match init_state: %f %f %f\n", init_state.pos(0),
           init_state.pos(1), init_state.pos(2));

    // 手动进行 IMU_init() 前的 kf 初始化 pose；
    // 原 IMU_init() 中会跳过第一帧，用于初始化 kf；其 pose 从 (0，0，0) 开始；
    this->GetAutoInitPose(init_state, Measures.lidar, this->map_);
    this->UpdateKfState(init_state);

    // 只查询 pose 附近 100 m 的点，避免在全局地图中查询；
    PointType point;
    point.x = init_state.pos(0);
    point.y = init_state.pos(1);
    point.z = init_state.pos(2);
    PointVector Storage;
    this->ikdtree.Radius_Search(point, dyn_map_radius_, Storage);
  }

  this->lidar_end_time = Measures.lidar_end_time;
  feats_undistort->clear();
  feats_undistort_filtered->clear();

  // 1. 预积分 + 去畸变
  p_imu->Process(Measures, kf, feats_undistort);

  for (size_t i = 0; i < feats_undistort->size(); i += point_filter_num) {
    feats_undistort_filtered->emplace_back(feats_undistort->points[i]);
  }
  // LOG(INFO) << "feats_undistort size: " << feats_undistort->size();
  // LOG(INFO) << "feats_undistort_filtered size: " << feats_undistort_filtered->size();

  // 2. 获取初始位姿
  state_point = kf.get_x();
  // update pos_lidar for 更新局部地图
  pos_lid =
      state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

  if ((feats_undistort_filtered == NULL) || feats_undistort_filtered->empty()) {
    std::cerr << "No point, skip this scan!\n" << std::endl;
    return;
  }

  flg_EKF_inited =
      (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

  // 2. 状态校准
  // auto frame_start = omp_get_wtime();
  this->Optimization(feats_undistort_filtered);
  // auto frame_end = omp_get_wtime();
  // std::cout << "Optimization runtime: " << (frame_end - frame_start) * 1000
  //           << " ms" << std::endl;

  /*** Debug variables ***/
  // std::cout << "ikdtree_dyn num: " << ikdtree_dyn.size() << " " << ikdtree_dyn.validnum()
  //           << std::endl;

  // for ouput loc pose
  o_pose.pos  = pos_lid;
  o_pose.rot  = state_point.rot * state_point.offset_R_L_I;
  pose_inited = true;
}

bool MapLocalization::Optimization(PointCloudXYZI::Ptr frame) {
  // auto& feats_undistort_filtered = frame;

  // ---------------- 地图检测 ----------------
  // 当前是否在地图外
  bool prev_outside           = enable_incremental_mapping_;
  enable_incremental_mapping_ = CheckOutsideGlobalMap();

  /*** initialize the dyn kdtree ***/
  // 状态切换：进入局部地图
  if (!prev_outside && enable_incremental_mapping_) {
    LOG(INFO) << "Switch to DYN MAP";

    // 无法像原 run_odometry() 跳过第一帧，因此先初始化 ikdtree_dyn
    this->InitDynMap();
  }

  /*** Segment the map in lidar FOV ***/
  // way 1 如果不使用全局先验地图，那么就需要在 ikdtree_dyn 中滑窗建立局部地图
  if (enable_incremental_mapping_) {
    lasermap_fov_segment();
    std::cerr << "lasermap_fov_segment() done!" << std::endl;
  }
  // way 2 在 kdtree 中 查询使用最近的局部地图；

  /*** downsample the feature points in a scan ***/
  downSizeFilterSurf.setInputCloud(frame);
  downSizeFilterSurf.filter(*feats_down_body);
  feats_down_size = feats_down_body->points.size();

  // LOG(INFO) << "feats_undistort size: " << frame->size();
  // LOG(INFO) << "feats_down_body size : " << feats_down_body->size();

  // 如果点云个数太少，跳过该帧
  if (feats_down_size < 5) {
    std::cout << "No point, skip this scan!" << std::endl;
    return false;
  }

  normvec->points.resize(feats_down_size);
  feats_down_world->points.resize(feats_down_size);

  Nearest_Points.resize(feats_down_size);  // 存储近邻点的vector

  /*** iterated state estimation ***/
  // auto start = omp_get_wtime();
  // cost 0.0002 ms
  auto& share = kf.get_dyn_share();

  share.user_ptr = this;

  share.feats_down_body  = feats_down_body;
  share.feats_down_world = feats_down_world;
  share.normvec          = normvec;
  share.laserCloudOri    = laserCloudOri;
  share.corr_normvect    = corr_normvect;

  share.Nearest_Points = &Nearest_Points;

  share.res_last            = res_last;
  share.point_selected_surf = point_selected_surf;

  share.feats_down_size = feats_down_size;

  double solve_H_time = 0;
  // 该更新过程中，选择 KDTree or KDTree_dyn
  // cost 46 ms in core thread 4
  kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
  // auto end = omp_get_wtime();
  // std::cout << "update_iterated_dyn_share_modified runtime: "
  //           << (end - start) * 1000 << " ms" << std::endl;

  // 获得最终更新匹配完的 pos_lidar
  state_point = kf.get_x();
  pos_lid =
      state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

  /*** initialize the outside map kdtree ***/
  // 超出地图时，需要增量更新地图
  if (enable_incremental_mapping_) {
    LOG(INFO) << "Outside GLOBAL map.";

    // way 1 把 ikdtree_dyn 直接附加到 ikdtree 上，拓展全局地图；
    // 此方式会污染 ikdtree，并且影响查询性能，放弃；

    /*** add the feature points to map kdtree_dyn ***/
    // way 2 双层地图：使用出图时刻的邻域构建 kdtree_dyn，然后增量该 kdtree_dyn
    // 在回到全局地图是，可合并，也可以丢弃；
    map_incremental();
  }

  // 状态切换：回到全局地图
  if (prev_outside && !enable_incremental_mapping_) {
    LOG(INFO) << "Back to GLOBAL MAP";
    // 暂时不用手动清空，交给 lasermap_fov_segment 自动删除
    // ikdtree_dyn.Clear();
    // 或者选择 merge
  }

  return true;
}

bool MapLocalization::CheckOutsideGlobalMap() {
  V3D p_global(state_point.rot * state_point.offset_T_L_I + state_point.pos);
  PointType query;
  query.x = p_global(0);
  query.y = p_global(1);
  query.z = p_global(2);

  double dist = this->IsFarFromMap(query);

  bool is_outside_measure = (dist > th_global_to_local);
  bool is_inside_measure  = (dist < th_local_to_global);

  // FSM 状态机
  switch (map_state_) {
    case MapState::INSIDE:
      if (is_outside_measure) {
        outside_counter++;
      } else {
        outside_counter = 0;
      }

      // 迟滞 + 连续帧稳定检测 判断是否稳定离开地图
      if (outside_counter >= stable_frame_num) {
        map_state_     = MapState::OUTSIDE;
        inside_counter = 0;
      }
      break;

    case MapState::OUTSIDE:
      if (is_inside_measure) {
        inside_counter++;
      } else {
        inside_counter = 0;
      }

      // 判断是否稳定回到地图
      if (inside_counter >= stable_frame_num) {
        map_state_      = MapState::INSIDE;
        outside_counter = 0;
      }
      break;
  }

  return (map_state_ == MapState::OUTSIDE);
}

double MapLocalization::IsFarFromMap(const PointType& pt) {
  // PointVector nearest_pts;
  KD_TREE<PointType>::PointVector nearest_pts;
  std::vector<float> sq_dist;

  ikdtree.Nearest_Search(pt, this->far_point_num_, nearest_pts, sq_dist);

  if (nearest_pts.empty() || nearest_pts.size() < this->far_point_num_) {
    return true;
  }

  double mean_dist = 0;

  for (int i = 0; i < this->far_point_num_; i++) {
    mean_dist += sqrt(sq_dist[i]);
  }
  mean_dist /= double(this->far_point_num_);

  return mean_dist;
}

void MapLocalization::Show(bool b_pause) {
  if (!pose_inited) return;

  static int cloud_id = 0;  // 点云编号
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr traj_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  static bool has_traj         = false;
  static std::string traj_name = "traj";

  if (vis == NULL) {
    vis.reset(new pcl::visualization::PCLVisualizer("vis pcd"));
    vis->setBackgroundColor(0, 0, 0);
    vis->initCameraParameters();
    // clang-format off
    vis->setCameraPosition(pos_lid[0], pos_lid[1], pos_lid[2] + 90, 
                           pos_lid[0], pos_lid[1], pos_lid[2], 
                           0, 0, 1);
    // clang-format on

    // 灰色
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        map_color_handler(map_, 128, 128, 128);
    vis->addPointCloud<PointType>(map_, map_color_handler, "init_map");
  }

  // 1. static cloud
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  vis_cloud->clear();
  vis_cloud->points.reserve(feats_down_world->size());

  // 2. nth_element instead of sort
  std::vector<float> z_list;
  z_list.reserve(feats_down_world->size());
  for (const auto& pt : feats_down_world->points) {
    z_list.push_back(pt.z);
  }

  size_t n       = z_list.size();
  size_t idx_min = n * 0.10;
  size_t idx_max = n * 0.90;
  std::nth_element(z_list.begin(), z_list.begin() + idx_min, z_list.end());
  float z_min_map = z_list[idx_min];
  std::nth_element(z_list.begin(), z_list.begin() + idx_max, z_list.end());
  float z_max_map = z_list[idx_max];

  std::cout << "[VIS] Map Z range: " << z_min_map << " ~ " << z_max_map
            << std::endl;

  // 3. build cloud
  float denom = std::max(1e-6f, z_max_map - z_min_map);
  for (const auto& pt : feats_down_world->points) {
    pcl::PointXYZRGB p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;

    // 归一化 z（使用 map 的范围）
    float z_norm = (p.z - z_min_map) / denom;

    // z_norm = std::clamp(z_norm, 0.0f, 1.0f);
    z_norm = std::min(1.0f, std::max(0.0f, z_norm));

    // 0.3 ~ 0.6 都很好用
    // float gamma = 0.4f;
    // z_norm = std::pow(z_norm, gamma);

    // 简单蓝→红渐变（稳定，不晃）
    auto bgr = GetColorByNorm(z_norm);
    // const auto& bgr = GetColorByNorm(z_norm);
    p.r = bgr(2);
    p.g = bgr(1);
    p.b = bgr(0);

    vis_cloud->push_back(p);
  }

  static std::string name = "current_cloud";

  if (vis->contains(name)) {
    vis->removePointCloud(name);
  }

  vis->addPointCloud<pcl::PointXYZRGB>(vis_cloud, name);
  vis->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

  pcl::PointXYZRGB traj;
  traj.x = pos_lid[0];
  traj.y = pos_lid[1];
  traj.z = pos_lid[2];
  traj.r = 0;
  traj.g = 255;
  traj.b = 255;
  traj_cloud->points.push_back(traj);

  // 只需要添加一次，之后只更新
  if (!has_traj) {
    vis->addPointCloud(traj_cloud, traj_name);
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, traj_name);
    has_traj = true;
  } else {
    vis->updatePointCloud(traj_cloud, traj_name);
  }

  if (b_pause) {
    vis->spin();
  } else {
    vis->spinOnce(1);
  }
}

void MapLocalization::ShowInitResult(pcl::visualization::PCLVisualizer::Ptr vis,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr map,
                                     const string& window_name) {
  if (vis == NULL) {
    vis.reset(new pcl::visualization::PCLVisualizer(window_name));
  }

  vis->setBackgroundColor(0, 0, 0);
  vis->setCameraPosition(-50, -50, 500, 0, 0, 0, 0, 0, 1);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      map_color_handler(map, 128, 128, 128);
  vis->addPointCloud<pcl::PointXYZ>(map, map_color_handler, "map cloud");
  vis->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map cloud");

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
      frame_color_handler(frame, "z");
  vis->addPointCloud<pcl::PointXYZ>(frame, frame_color_handler, "frame cloud");
  vis->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "frame cloud");

  vis->addCoordinateSystem(10.0);
  vis->spin();

  vis->removeAllPointClouds();
  vis->removeAllShapes();
  vis->close();
}

void MapLocalization::ShowMatchResultDual(
    pcl::visualization::PCLVisualizer::Ptr vis,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map, const string& window_name) {
  if (vis == NULL) {
    vis.reset(new pcl::visualization::PCLVisualizer(window_name));
  }

  // | 左 (v1) | 右 (v2) |
  int v1(0);
  int v2(1);
  vis->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  vis->createViewPort(0.5, 0, 1.0, 1.0, v2);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> current_h1(
      cloud_first, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> current_h2(
      cloud_second, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> last_h(
      cloud_map, 0, 255, 0);

  vis->addPointCloud(cloud_first, current_h1, "vp2_source1", v1);
  vis->addPointCloud(cloud_map, last_h, "vp2_target1", v1);
  vis->addPointCloud(cloud_second, current_h2, "vp2_source2", v2);
  vis->addPointCloud(cloud_map, last_h, "vp2_target2", v2);

  vis->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_source1");
  vis->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_source2");
  vis->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_target1");
  vis->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_target2");

  // vis->addCoordinateSystem(10.0);
  vis->spin();

  vis->removeAllPointClouds();
  vis->removeAllShapes();
  vis->close();
}

void MapLocalization::GetWholeMap(PointCloudXYZI::Ptr& cloud_map) {
  if (cloud_map == NULL) {
    cloud_map.reset(new PointCloudXYZI);
  }
  assert(cloud_map != nullptr);

  // 不包含 ikdtree_dyn；单线程使用
  ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);

  // 局部存储 多线程安全
  // std::vector<PointType> local_storage;
  // ikdtree.flatten(ikdtree.Root_Node, local_storage, NOT_RECORD);

  cloud_map->clear();
  cloud_map->points.reserve(ikdtree.PCL_Storage.size() + _featsArray->size());

  // 拷贝 map
  cloud_map->points.insert(cloud_map->points.end(), ikdtree.PCL_Storage.begin(),
                           ikdtree.PCL_Storage.end());

  // 拷贝 feats
  if (_featsArray) {
    cloud_map->points.insert(cloud_map->points.end(), _featsArray->points.begin(),
                            _featsArray->points.end());
  }
}

}  // namespace fastlio
