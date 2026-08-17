#include "modules/localization/fast_lio/lidar_odometry.h"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

namespace fastlio {

LidarOdometry::LidarOdometry() {
  feats_undistort.reset(new PointCloudXYZI());
  this->feats_down_body.reset(new PointCloudXYZI());
  this->feats_down_world.reset(new PointCloudXYZI());
  this->normvec.reset(new PointCloudXYZI(100000, 1));
  this->laserCloudOri.reset(new PointCloudXYZI(100000, 1));
  this->corr_normvect.reset(new PointCloudXYZI(100000, 1));
  p_imu = std::make_shared<ImuProcess>();

  // 如果没有回环检测需求，可以注释
  _featsArray.reset(new PointCloudXYZI(100000, 1));

  feats_undistort_filtered.reset(new PointCloudXYZI());

  local_map_cloud_.reset(new PointCloudXYZI());
  traj_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
  // std::cout << "MP_EN ON" << std::endl;
#endif
}

LidarOdometry::~LidarOdometry() {}

void LidarOdometry::SetGravityImuExtrinsicMatrix(
    const Eigen::Matrix4f& extrinsic_matrix) {
  /* debug 雷达-IMU 非正装
  // clang-format off
  Eigen::Matrix4d imu_ext = Eigen::Matrix4d::Identity();
  // imu_ext << 1.0, 0.0, 0.0, 0.0, 
  //            0.0, 0.6428, -0.7660, 0.0, 
  //            0.0, 0.7660, 0.6428, 0.0, 
  //            0.0, 0.0, 0.0, 1.0;
  imu_ext << 1.0, 0.0, 0.0, 0.0,
             0.0, -1.0, 0.0, 0.0,
             0.0, 0.0, -1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
  // clang-format on
  */
  gravity_imu_ext = extrinsic_matrix.cast<double>();

  // std::cout << "gravity_imu_ext: " << std::endl << gravity_imu_ext << std::endl;
  // abort();
}

void LidarOdometry::SetExtrinsicMatrix(
    const Eigen::Matrix4f& extrinsic_matrix) {
  // 转成 double
  // Eigen::Matrix4d ext = extrinsic_matrix.cast<double>();
  Eigen::Matrix4d ext = extrinsic_matrix.cast<double>() * gravity_imu_ext;

  // 提取旋转
  Lidar_R_wrt_IMU = ext.block<3, 3>(0, 0);
  // 提取平移
  Lidar_T_wrt_IMU = ext.block<3, 1>(0, 3);
  // std::cout << Lidar_R_wrt_IMU << std::endl;
  // std::cout << Lidar_T_wrt_IMU << std::endl;
}

void LidarOdometry::Init(
    std::shared_ptr<jojo::localization::RuntimeConfig> param) {
  rparam_ = param;
  this->SetDataFolder();

  /*** variables definition ***/
  memset(point_selected_surf, true, sizeof(point_selected_surf));
  // memset 无法按预期把 float 数组全部设置为 -1000.0f。
  // 如果直接这么运行，你的 res_last 数组会被填充成一个极其奇怪的垃圾浮点数值。
  // memset(res_last, -1000.0f, sizeof(res_last));
  // ==> std::fill
  std::fill_n(res_last, 100000, -1000.0f);
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

void LidarOdometry::Init(
    std::shared_ptr<jojo::localization::RuntimeConfig> rparam,
    std::shared_ptr<jojo::localization::StaticConfig> sparam) {
  sparam_ = sparam;
  if (!this->InitStaticConfig()) {
    throw std::invalid_argument("StaticConfig must not be null");
  }
  this->Init(rparam);
}

bool LidarOdometry::InitStaticConfig() {
  if (!sparam_) {
    return false;
  }

  lidar_type       = sparam_->lidar_type;
  point_filter_num = sparam_->point_filter_num;

  gyr_cov   = sparam_->gyr_cov;
  acc_cov   = sparam_->acc_cov;
  b_gyr_cov = sparam_->b_gyr_cov;
  b_acc_cov = sparam_->b_acc_cov;

  DET_RANGE = sparam_->DET_RANGE;
  cube_len  = sparam_->cube_len;

  filter_size_surf_min = sparam_->filter_size_surf_min;
  filter_size_map_min  = sparam_->filter_size_map_min;

  extrinsic_est_en   = sparam_->extrinsic_est_en;
  NUM_MAX_ITERATIONS = sparam_->NUM_MAX_ITERATIONS;

  return true;
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
  // 初始化局部地图范围，以 pos_LiD 为中心,长宽高均为 cube_len
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

/* for ros msg publish
bool LidarOdometry::sync_packages(MeasureGroup& meas) {
  // double lidar_mean_scantime = 0.0;
  // int scan_num = 0;

  return true;
}
*/

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

  /* way 1 源代码，能通，但有潜在语法问题，因此升级为下面的版本
  // laserCloudOri->clear();  // 计算点-面残差时，实际用到的满足要求的点坐标，l系
  // corr_normvect->clear();  // 计算点-面残差时，实际用到的点对应平面的参数，w系
  */

  // debug ==> std::out_of_range
  // laserCloudOri->points.at(0) = feats_down_body->points.at(0);

  // !! 为什么源代码不跳出？
  // clear() 只把逻辑元素数量设为0，通常不会释放已经申请的 100000 个点的内存。
  // 运行时，std::vector::operator[] 基本等价于 return *(date_pointer + index);
  // 只要 effect_feat_num < 100000，写入地址仍然落在 vector 已申请的那块堆内存中
  // 后续读取时，同样使用不检查边界的 operator[]，因此不会出现越界访问，能读出刚刚写进去的数据。
  // 而 laserCloudOri->points.at(0) 会严格检查 0 < points.size();

  // /* way2 有效点会在并行区结束后按索引压缩到这两个缓存。
  // 先定长，保证 operator[] 写入合法；已有 capacity 足够时 resize 不会重新分配。
  laserCloudOri->resize(feats_down_size);
  corr_normvect->resize(feats_down_size);
  // */

  total_residual = 0.0;

  /** closest surface search and residual computation **/

#ifdef MP_EN
#pragma omp parallel
  {
    // std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
    thread_local std::vector<float> pointSearchSqDis;
    pointSearchSqDis.clear();
    pointSearchSqDis.reserve(NUM_MATCH_POINTS);

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


      // auto& points_near = Nearest_Points[i];
      // zero copy
      auto& points_near = (*Nearest_Points)[i];
      // way 1 源代码
      // points_near.clear();

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
  }
#else
  std::cout << "MP_EN is OFF" << std::endl;
  std::cerr << "Lidar Odometry must enable MP_EN for performance 100ms !!"
            << std::endl;
  abort();
#endif

  effct_feat_num = 0;

  for (int i = 0; i < feats_down_size; i++) {
    if (point_selected_surf[i]) {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      // !! 不支持并行
      // laserCloudOri->push_back(feats_down_body->points[i]);
      // corr_normvect->push_back(normvec->points[i]);
      total_residual += res_last[i];
      effct_feat_num++;
    }
  }

  // /* way 2
  laserCloudOri->resize(effct_feat_num);
  corr_normvect->resize(effct_feat_num);
  // */

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

bool LidarOdometry::run_odometry(MeasureGroup& Measures) {
  // /*
  if (flg_first_scan) {
    first_lidar_time        = Measures.lidar_beg_time;
    p_imu->first_lidar_time = first_lidar_time;
    flg_first_scan          = false;
    p_imu->SetInitMode(0);
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

  // 0. 将 IMU 数据变换到 正装坐标系；
  // 也许数据生成时就变换挺好的，但是会污染 原始 IMU 数据
  this->TransformImuData(Measures);

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
    return false;
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

      return false;
    }
  }
  // int featsFromMapNum = ikdtree.validnum();
  // kdtree_size_st = ikdtree.size();
  // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

  /*** ICP and iterated Kalman filter update ***/
  if (feats_down_size < 5) {
    std::cout << "No point, skip this scan!" << std::endl;
    return false;
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

  this->Nearest_Points.resize(feats_down_size);  // 存储近邻点的vector
  // int rematch_num        = 0;
  // bool nearest_search_en = true;

  // auto t2 = omp_get_wtime();

  /*** iterated state estimation ***/
  // 传递 更新后的数据
  auto& share = kf.get_dyn_share();

  share.user_ptr = this;

  // 将 LidarOdometry 的内部成员传递给 share
  share.feats_down_body  = this->feats_down_body;
  share.feats_down_world = this->feats_down_world;
  share.normvec          = this->normvec;
  share.laserCloudOri    = this->laserCloudOri;
  share.corr_normvect    = this->corr_normvect;

  share.Nearest_Points = &(this->Nearest_Points);

  share.res_last            = this->res_last;
  share.point_selected_surf = this->point_selected_surf;

  share.feats_down_size = this->feats_down_size;

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

  return true;
}

void LidarOdometry::TransformImuData(MeasureGroup& measures) {
  const Eigen::Matrix3d R = gravity_imu_ext.block<3, 3>(0, 0);

  for (auto& imu : measures.imu) {
    imu.linear_acceleration = R * imu.linear_acceleration;
    imu.angular_velocity    = R * imu.angular_velocity;
  }
}

void LidarOdometry::SetDataFolder() {
  this->prefix = rparam_->root_path + "/" + rparam_->file_name;
  // std::cout << "data_file : " << this->prefix << std::endl;

  this->postfix = this->prefix + "-O";
  path_lidar = this->postfix + "/sensor_data/" + "lidar_pcd";
  std::error_code directory_error;
  std::filesystem::create_directories(path_lidar, directory_error);
  if (directory_error) {
    throw std::runtime_error(
        "cannot create Fast-LIO output directory '" + path_lidar +
        "': " + directory_error.message());
  }

  path_pose = this->postfix + "/sensor_data/" + "pose" + ".txt";
  ofs_pose.open(path_pose, std::ios::out);
  if (!ofs_pose.is_open()) {
    throw std::runtime_error(
        "cannot open Fast-LIO pose output '" + path_pose + "'");
  }

  if (rparam_->b_only_times) {
    path_runtime = this->postfix + "/sensor_data/" + "runtime" + ".txt";
    // 以追加方式打开（不存在会自动创建）
    ofs_runtime.open(path_runtime, std::ios::app);
    if (!ofs_runtime.is_open()) {
      throw std::runtime_error(
          "cannot open Fast-LIO runtime output '" + path_runtime + "'");
    }
  }
}

void LidarOdometry::Close() {
  ofs_pose.close();

  if (rparam_->b_only_times) {
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
             << o_pose.pos.x() << " " << o_pose.pos.y() << " " << o_pose.pos.z()
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
    oss_time << std::setw(13) << std::setfill('0')
             << static_cast<uint64_t>(lidar_end_time * 1000);
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
  if (!pose_inited || ikdtree.Root_Node == nullptr) return;

  const std::string traj_name = "traj";
  const std::string lm_name   = "local_map";
  const std::string lmb_name  = "local_map_bounds";

  if (vis == NULL) {
    vis.reset(new pcl::visualization::PCLVisualizer(
        "FAST-LIO local map (gray) / current scan (yellow)"));
    vis->setBackgroundColor(0.03, 0.03, 0.03);
    vis->initCameraParameters();
    vis->setCameraPosition(pos_lid[0] - 30.0, pos_lid[1] - 30.0,
                           pos_lid[2] + 25.0, pos_lid[0], pos_lid[1],
                           pos_lid[2], 0.0, 0.0, 1.0);
    vis->addCoordinateSystem(2.0, "world_axis");
  }

  if (vis->wasStopped()) return;

  const std::string name = "current_cloud";

  // ikd-tree 是实际参与匹配的滑动局部地图。
  // 首帧以及之后每 N 帧展开一次地图显示，避免每帧 O(map_size) 的复制拖慢里程计。
  const bool refresh_local_map =
      !local_map_added_to_viewer_ ||
      visualization_frame_count_ % local_map_refresh_period_ == 0;

  if (refresh_local_map) {
    // clang-format off
    local_map_cloud_->clear();
    local_map_cloud_->points.reserve(static_cast<std::size_t>(std::max(0, ikdtree.validnum())));
    ikdtree.flatten(ikdtree.Root_Node, local_map_cloud_->points, NOT_RECORD);

    local_map_cloud_->width = static_cast<std::uint32_t>(local_map_cloud_->points.size());
    local_map_cloud_->height = 1;
    local_map_cloud_->is_dense = false;
    // clang-format on

    pcl::visualization::PointCloudColorHandlerCustom<PointType> map_color(
        local_map_cloud_, 150, 150, 150);
    if (!local_map_added_to_viewer_) {
      local_map_added_to_viewer_ =
          vis->addPointCloud<PointType>(local_map_cloud_, map_color, lm_name);
      if (local_map_added_to_viewer_) {
        vis->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, lm_name);
      }
    } else {
      vis->updatePointCloud<PointType>(local_map_cloud_, map_color, lm_name);
    }

    // 同步显示 FAST-LIO 当前滑动地图的边界。
    if (vis->contains(lmb_name)) {
      vis->removeShape(lmb_name);
    }
    vis->addCube(LocalMap_Points.vertex_min[0], LocalMap_Points.vertex_max[0],
                 LocalMap_Points.vertex_min[1], LocalMap_Points.vertex_max[1],
                 LocalMap_Points.vertex_min[2], LocalMap_Points.vertex_max[2],
                 0.2, 0.7, 1.0, lmb_name);
    vis->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, lmb_name);
  }

  // 当前帧每次刷新，并用亮黄色与灰色局部地图区分。
  pcl::visualization::PointCloudColorHandlerCustom<PointType> scan_color(
      feats_down_world, 255, 220, 40);
  if (!current_scan_added_to_viewer_) {
    vis->addPointCloud<PointType>(feats_down_world, scan_color, name);
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    current_scan_added_to_viewer_ = true;
  } else {
    vis->updatePointCloud<PointType>(feats_down_world, scan_color, name);
  }

  pcl::PointXYZRGB traj;
  traj.x = pos_lid[0];
  traj.y = pos_lid[1];
  traj.z = pos_lid[2];
  traj.r = 0;
  traj.g = 255;
  traj.b = 255;
  traj_cloud->points.push_back(traj);
  traj_cloud->width  = static_cast<std::uint32_t>(traj_cloud->points.size());
  traj_cloud->height = 1;

  if (!trajectory_added_to_viewer_) {
    vis->addPointCloud(traj_cloud, traj_name);
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, traj_name);
    trajectory_added_to_viewer_ = true;
  } else {
    vis->updatePointCloud(traj_cloud, traj_name);
  }

  ++visualization_frame_count_;

  if (b_pause) {
    vis->spin();
  } else {
    vis->spinOnce(1);
  }
}

}  // namespace fastlio
