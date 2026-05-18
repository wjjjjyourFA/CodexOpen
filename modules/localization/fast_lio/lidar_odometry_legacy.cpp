#include "modules/localization/fast_lio/lidar_odometry_legacy.h"

namespace fastlio {
namespace common = apollo::cyber::common;

LidarOdometry::LidarOdometry() {
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  // normvec.reset(new PointCloudXYZI(100000, 1));
  p_imu = std::make_shared<ImuProcess>();

  feats_undistort_filtered.reset(new PointCloudXYZI());
}

LidarOdometry::~LidarOdometry() {}

void LidarOdometry::SetExtrinsicMatrix(
    const Eigen::Matrix4f& extrinsic_matrix) {
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
  // memset(point_selected_surf, true, sizeof(point_selected_surf));
  // memset(res_last, -1000.0f, sizeof(res_last));
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
  // double epsi[23] = {0.001};
  // fill(epsi, epsi+23, 0.001);
  // kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
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
  // for (int i = 0; i < points_history.size(); i++) {
  //   _featsArray->push_back(points_history[i]);
  // }
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

  // normvec->points.resize(feats_down_size);
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
  // auto t_update_start = omp_get_wtime();
  double solve_H_time = 0;
  // “LiDAR观测 → 状态校准”：基于点云残差的迭代EKF更新（IEKF）
  // kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
  kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body,
                                        ikdtree, Nearest_Points,
                                        NUM_MAX_ITERATIONS, extrinsic_est_en);

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
  o_pose.rot  = state_point.rot.unit_quaternion() *
                state_point.offset_R_L_I.unit_quaternion();
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
}

void LidarOdometry::Close() { ofs_pose.close(); }

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

void LidarOdometry::Show(bool b_pause) {
  if (!pose_inited) return;

  static int cloud_id = 0;  // 点云编号
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr traj_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  static bool has_traj  = false;
  std::string traj_name = "traj";

  if (vis == NULL) {
    vis = new pcl::visualization::PCLVisualizer("vis pcd");
    vis->setBackgroundColor(0, 0, 0);
    vis->initCameraParameters();
    vis->setCameraPosition(0, -20, 10, 0, 0, 1);
  }

  std::string name = "current_cloud";
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
