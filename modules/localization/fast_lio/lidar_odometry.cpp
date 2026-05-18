#include "modules/localization/fast_lio/lidar_odometry.h"

namespace fastlio {
namespace common = apollo::cyber::common;

LidarOdometry::LidarOdometry() {
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  normvec.reset(new PointCloudXYZI(100000, 1));
  laserCloudOri.reset(new PointCloudXYZI(100000, 1));
  corr_normvect.reset(new PointCloudXYZI(100000, 1));
  p_imu = std::make_shared<ImuProcess>();

  // 如果没有显示需求，可以注释
  _featsArray.reset(new PointCloudXYZI(100000, 1));

  feats_undistort_filtered.reset(new PointCloudXYZI());
}

LidarOdometry::~LidarOdometry() {}

void LidarOdometry::SetExtrinsicMatrix(
    const Eigen::Matrix4f& extrinsic_matrix) {
  /* debug
  // clang-format off
  Eigen::Matrix4d imu_ext = Eigen::Matrix4d::Identity();
  imu_ext << 1.0, 0.0, 0.0, 0.0, 
             0.0, 0.6428, -0.7660, 0.0, 
             0.0, 0.7660, 0.6428, 0.0, 
             0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix4d ext = imu_ext * extrinsic_matrix.cast<double>();
  // clang-format on
  */

  // 转成 double
  Eigen::Matrix4d ext = extrinsic_matrix.cast<double>();
  // 提取旋转
  Lidar_R_wrt_IMU = ext.block<3, 3>(0, 0);
  // 提取平移
  Lidar_T_wrt_IMU = ext.block<3, 1>(0, 3);
  // std::cout << Lidar_R_wrt_IMU << std::endl;
  // std::cout << Lidar_T_wrt_IMU << std::endl;
}

void LidarOdometry::Init(
    std::shared_ptr<jojo::localization::RuntimeConfig> param) {
  param_ = param;
  this->SetDataFolder();

  /*** variables definition ***/
  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
                                filter_size_map_min);

  // goto: SetExtrinsicMatrix()
  // Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  // Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  // p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_whole_param(
      Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov),
      V3D(acc_cov, acc_cov, acc_cov), V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov),
      V3D(b_acc_cov, b_acc_cov, b_acc_cov));
  p_imu->lidar_type = lidar_type;
  // kf 重构版，直接在调用时，指定相关参数
  // 初始化一个“迭代卡尔曼滤波器”，并设置每个状态维度的收敛精度，用于控制 IEKF 的迭代停止条件。
  double epsi[23] = {0.001};
  fill(epsi, epsi + 23, 0.001);
  // clang-format off
  // 注册“系统模型 + 观测模型”的函数指针 ==> h_share_model 点云匹配函数
  // kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
  kf.init_dyn_share(get_f, df_dx, df_dw, LidarOdometry::h_share_model_static, NUM_MAX_ITERATIONS, epsi);
  // clang-format on
}

void LidarOdometry::pointBodyToWorld(PointType const* const pi,
                                     PointType* const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot.matrix() *
                   (state_point.offset_R_L_I.matrix() * p_body +
                    state_point.offset_T_L_I) +
               state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  // no use
  po->intensity = pi->intensity;
}

void LidarOdometry::points_cache_collect() {
  PointVector points_history;
  ikdtree.acquire_removed_points(points_history);
  // 可以用来拼接做全局地图（离线）|| 用于回环检测
  for (int i = 0; i < points_history.size(); i++) {
    _featsArray->push_back(points_history[i]);
  }
}

void LidarOdometry::lasermap_fov_segment() {
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
    // 第一帧时，刚好不会触发到这里，已经提前退出
    kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
  }
  // kdtree_delete_time = omp_get_wtime() - delete_begin;
}

bool LidarOdometry::sync_packages(MeasureGroup& meas) {
  double lidar_mean_scantime = 0.0;
  int scan_num               = 0;
}

void LidarOdometry::map_incremental() {
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
  add_point_size = ikdtree.Add_Points(PointToAdd, true);
  add_point_size += ikdtree.Add_Points(PointNoNeedDownsample, false);
  // ikdtree.Add_Points(PointNoNeedDownsample, false);
  // add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
  // kdtree_incremental_time = omp_get_wtime() - st_time;

  // std::cout << "add num: " << PointToAdd.size() << "  "
  //           << PointNoNeedDownsample.size() << " " << add_point_size
  //           << std::endl;
  // std::cout << "ikdtree num: " << ikdtree.size() << " " << ikdtree.validnum()
  //           << std::endl;
}

void LidarOdometry::h_share_model(
    state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
  // 构建 观测模型（measurement model），用于计算残差 ==> ICP 的残差 + Jacobian
  // 点到地图的几何约束：点云（当前帧） → 对齐 → 地图
  // 最小化：点到平面的距离

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
#pragma omp parallel for
#endif
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

    std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    // auto& points_near = Nearest_Points[i];
    // zero copy
    auto& points_near = (*Nearest_Points)[i];
    points_near.clear();

    if (ekfom_data.converge) {
      /** Find the closest surfaces in the map **/
      // 偏 优化约束
      ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
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
}

void LidarOdometry::run_odometry(MeasureGroup& Measures) {
  // /*
  if (flg_first_scan) {
    first_lidar_time        = Measures.lidar_beg_time;
    p_imu->first_lidar_time = first_lidar_time;
    flg_first_scan          = false;
  }
  // */

  // match_time = 0;
  // kdtree_search_time = 0.0;
  // solve_time = 0;
  // solve_const_H_time = 0;
  // svd_time = 0;
  // auto t1 = omp_get_wtime();

  this->lidar_end_time = Measures.lidar_end_time;
  feats_undistort->clear();
  feats_undistort_filtered->clear();

  // 1. 预积分 + 去畸变
  p_imu->Process(Measures, kf, feats_undistort);

  // add filter PointCloud
  // 工程化降采样：
  // 如果有大量的空点，该采样方法反而会丢失有效点云，因此输入点云已经去除零点；
  // 与 pcl::VoxelGrid 相比，该采样方法在速度上更快，同时不会丢失点云密度；
  /*
  for (size_t i = 0; i < feats_undistort->size(); i++) {
    if (i % point_filter_num == 0) {
      feats_undistort_filtered->emplace_back(feats_undistort->points[i]);
    }
  }
  */
  for (size_t i = 0; i < feats_undistort->size(); i += point_filter_num) {
    feats_undistort_filtered->emplace_back(feats_undistort->points[i]);
  }

  // 2. 获取初始位姿
  state_point = kf.get_x();
  // update pos_lidar for 更新局部地图
  // pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
  pos_lid =
      state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

  if ((feats_undistort_filtered == NULL) || feats_undistort_filtered->empty()) {
    std::cerr << "No point, skip this scan!\n" << std::endl;
    return;
  }

  flg_EKF_inited =
      (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

  /*** Segment the map in lidar FOV ***/
  // 20260401：该函数作用是什么？
  // 维护一个“跟着雷达走的局部地图（Local Map）”
  // 控制 KD-tree 规模，在局部达到性能和精度的平衡
  lasermap_fov_segment();

  /*** downsample the feature points in a scan ***/
  // feats_down_body 之前的数据 会被自动清空
  // downSizeFilterSurf.setLeafSize(0.1f, 0.1f, 0.1f);
  downSizeFilterSurf.setInputCloud(feats_undistort_filtered);
  downSizeFilterSurf.filter(*feats_down_body);
  // auto t1 = omp_get_wtime();
  feats_down_size = feats_down_body->points.size();

  /*** initialize the map kdtree ***/
  // 第一帧：初始化地图（没有观测更新）
  if (ikdtree.Root_Node == nullptr) {
    if (feats_down_size > 5) {
      ikdtree.set_downsample_param(filter_size_map_min);
      feats_down_world->points.resize(feats_down_size);
      for (int i = 0; i < feats_down_size; i++) {
        // lidar坐标系转到世界坐标系
        pointBodyToWorld(&(feats_down_body->points[i]),
                         &(feats_down_world->points[i]));
      }
      // 根据世界坐标系下的点构建ikdtree
      ikdtree.Build(feats_down_world->points);

      return;
    }
  }
  // int featsFromMapNum = ikdtree.validnum();
  // kdtree_size_st = ikdtree.size();
  // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

  /*** ICP and iterated Kalman filter update ***/
  if (feats_down_size < 5) {
    std::cout << "No point, skip this scan!" << std::endl;
    return;
  }

  normvec->points.resize(feats_down_size);
  feats_down_world->points.resize(feats_down_size);

  /* If you need to see map point, change to "if(1)"
  if (0) {
    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    featsFromMap->clear();
    featsFromMap->points = ikdtree.PCL_Storage;
  }
  */

  Nearest_Points.resize(feats_down_size);  // 存储近邻点的vector
  // int rematch_num        = 0;
  // bool nearest_search_en = true;

  // auto t2 = omp_get_wtime();

  /*** iterated state estimation ***/
  // 传递 更新后的数据
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

  // auto t_update_start = omp_get_wtime();
  double solve_H_time = 0;
  // “LiDAR观测 → 状态校准”：基于点云残差的迭代EKF更新（IEKF）
  // 开始 EKF / ICP 优化：在 kf 内部，迭代调用 init_dyn_share() 注册的函数，优化状态
  kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);

  // 获得最终更新匹配完的 pos_lidar
  state_point = kf.get_x();
  // pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
  pos_lid =
      state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

  // auto t_update_end = omp_get_wtime();

  /*** add the feature points to map kdtree ***/
  // auto t3 = omp_get_wtime();
  map_incremental();
  // auto t5 = omp_get_wtime();

  /*** Debug variables ***/
  // std::cout << "ikdtree num: " << ikdtree.size() << " " << ikdtree.validnum()
  //           << std::endl;

  // for ouput odom pose
  o_pose.pos  = pos_lid;
  o_pose.rot  = state_point.rot * state_point.offset_R_L_I;
  pose_inited = true;
}

void LidarOdometry::SetDataFolder() {
  this->prefix = param_->root_path + "/" + param_->file_name;
  // std::cout << "data_file : " << this->prefix << std::endl;

  this->postfix = this->prefix + "-O";

  path_lidar = this->postfix + "/sensor_data/" + "lidar_pcd";
  common::CreateDir(path_lidar);

  path_pose = this->postfix + "/sensor_data/" + "pose" + ".txt";
  ofs_pose.open(path_pose, std::ios::out);
  if (!ofs_pose.is_open()) {
    std::cerr << "[save_result] Cannot clear " << path_pose << std::endl;
  }

  if (param_->b_only_times) {
    path_runtime = this->postfix + "/sensor_data/" + "runtime" + ".txt";
    // 以追加方式打开（不存在会自动创建）
    ofs_runtime.open(path_runtime, std::ios::app);
    if (!ofs_runtime.is_open()) {
      std::cerr << "❌ Failed to open file: " << path_runtime << std::endl;
    }
  }
}

void LidarOdometry::Close() {
  ofs_pose.close();

  if (param_->b_only_times) {
    ofs_runtime.close();
  }
}

void LidarOdometry::save_result(bool b_save_pcd) {
  if (!pose_inited) {
    pose_init_count++;
    return;
  }

  // way 1 (不推荐) 欧拉角的解法不固定，不怎么平滑
  // Eigen::Vector3d euler = o_pose.rot.toRotationMatrix().eulerAngles(2, 1, 0);

  // way 2
  Eigen::Vector3d euler;
  Eigen::Matrix3d rotation = o_pose.rot.toRotationMatrix();
  jojo::common::transform::RotationToEulerZYX(rotation, euler);

  // 写入位姿信息
  if (ofs_pose.is_open()) {
    // timestamp x y z r p y
    ofs_pose << std::fixed << uint64_t(lidar_end_time * 1000) << " "
             << o_pose.pos.x() << " " << o_pose.pos.y() << " "
             << o_pose.pos.z()
             << " " << euler[2] << " " << euler[1] << " " << euler[0]
             << std::endl;
  } else {
    std::cerr << "[save_result] Cannot open pose.txt!" << std::endl;
  }

  // 保存点云
  if (b_save_pcd && !feats_undistort->empty()) {
    feats_undistort->width  = feats_undistort->points.size();
    feats_undistort->height = 1;

    std::ostringstream oss_time;
    oss_time << std::fixed << std::setprecision(13)
             << uint64_t(lidar_end_time * 1000);
    std::string pcd_filename = path_lidar + "/" + oss_time.str() + ".pcd";
    pcl::io::savePCDFileBinary(pcd_filename, *feats_undistort);
    // std::cout << "Saving " << pcd_filename << std::endl;
  }
}

void LidarOdometry::SaveFrameTime(double time_ms) {
  // 保存每帧耗时（单位：ms）

  // 保留三位小数
  ofs_runtime << std::fixed << std::setprecision(3) << time_ms << std::endl;
}

void LidarOdometry::Show(bool b_pause) {
  if (!pose_inited) return;

  static int cloud_id = 0;  // 点云编号
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr traj_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  static bool has_traj         = false;
  static std::string traj_name = "traj";

  if (vis == NULL) {
    // vis = boost::make_shared<pcl::visualization::PCLVisualizer>("vis pcd");
    vis.reset(new pcl::visualization::PCLVisualizer("vis pcd"));
    vis->setBackgroundColor(0, 0, 0);
    vis->initCameraParameters();
    vis->setCameraPosition(0, -20, 10, 0, 0, 1);
  }

  static std::string name = "current_cloud";
  // 添加当前点云（按帧编号叠加）
  // std::string name = "cloud_" + std::to_string(cloud_id++);
  // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(feats_down_world, "z");
  // vis->addPointCloud<pcl::PointXYZI>(feats_down_world, intensity_color, name);
  // vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

  // 每帧更新点云：删除上一帧的点云，只保留当前帧
  if (vis->contains(name)) {
    vis->removePointCloud(name);
  }

  pcl::visualization::PointCloudColorHandlerGenericField<PointType>
      color_handler(feats_down_world, "z");
  vis->addPointCloud<PointType>(feats_down_world, color_handler, name);
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

}  // namespace fastlio
