
#include "MapProcess.h"







MapProcess::MapProcess()
{



}

MapProcess::~MapProcess()
{

}




// 初始化函数
bool MapProcess::InitConfig(const std::string &config_yaml)
{


    string root_dir = ROOT_DIR;


    // YAML::Node config;
    // try{
    //     config = YAML::LoadFile(config_yaml);
    // }
    // catch(YAML::BadFile &e){
    //     std::cerr << "read error config yaml !" << config_yaml <<std::endl;
    // }

    // std::cout<<"sucess read config from  " << config_yaml << std::endl;

    // path_pub_en      = config["publish"]["path_en"].as<bool>(true);
    // scan_pub_en      = config["publish"]["scan_pub_en"].as<bool>(true);
    // dense_pub_en     = config["publish"]["dense_pub_en"].as<bool>(true);
    // scan_body_pub_en = config["publish"]["scan_bodyframe_pub_en"].as<bool>(true);

//    lid_topic               = config["common"]["lid_topic"].as<std::string>("/sensor/RS128Points");
//    imu_topic               = config["common"]["imu_topic"].as<std::string>("/livox/imu");
//    imuRaw_topic            = config["common"]["imuRaw_topic"].as<std::string>("/sensor/Imu");
//    globalPose_topic        = config["common"]["globalPose_topic"].as<std::string>("/self_state/GlobalPose");
//    lidarLocalPose_topic    = config["common"]["lidarLocalPose_topic"].as<std::string>("/self_state/LidarLocalPose");
    // time_sync_en            = config["common"]["time_sync_en"].as<bool>(false);
    // time_diff_lidar_to_imu  = config["common"]["time_offset_lidar_to_imu"].as<double>(0.0);



    // p_imu->gyr_cov          = config["mapping"]["gyr_cov"].as<double>(0.1);
    // p_imu->acc_cov          = config["mapping"]["acc_cov"].as<double>(0.1);
    // p_imu->b_gyr_cov        = config["mapping"]["b_gyr_cov"].as<double>(0.0001);
    // p_imu->b_acc_cov        = config["mapping"]["b_acc_cov"].as<double>(0.0001);
    // extrinsic_est_en        = config["mapping"]["extrinsic_est_en"].as<bool>(true);
    // DET_RANGE               = config["mapping"]["det_range"].as<float>(300.f);
    // cube_len                = config["mapping"]["cube_side_length"].as<double>(200);
    // filter_size_surf_min    = config["mapping"]["filter_size_surf"].as<double>(0.5);
    // filter_size_map_min     = config["mapping"]["filter_size_map"].as<double>(0.5);
    // NUM_MAX_ITERATIONS      = config["mapping"]["max_iteration"].as<int>(4);
    // extrinT                 = config["mapping"]["extrinsic_T"].as<std::vector<double>>(std::vector<double>());
    // extrinR                 = config["mapping"]["extrinsic_R"].as<std::vector<double>>(std::vector<double>());


    // runtime_pos_log     = config["pcd_save"]["runtime_pos_log_enable"].as<bool>(0);



    // std::cout<< config<<std::endl;



    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    // Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    // p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    // p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    // p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    // p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    // p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);


    cloud_map.reset(new PointCloudXYZI());

    featsFromMap.reset(new PointCloudXYZI());
    feats_undistort.reset(new PointCloudXYZI());
    feats_undistort_world.reset(new PointCloudXYZI());
    

    feats_down_body.reset(new PointCloudXYZI());
    feats_down_world.reset(new PointCloudXYZI());
    normvec.reset(new PointCloudXYZI(100000, 1));
    laserCloudOri.reset(new PointCloudXYZI(100000, 1));
    corr_normvect.reset(new PointCloudXYZI(100000, 1));
    pcl_wait_save.reset(new PointCloudXYZI());
    _featsArray.reset(new PointCloudXYZI());    // pcl::PointCloud<PointType>   pcl::PointXYZINormal
    ikdtree_cloud.reset(new PointCloudXYZI());  



//    lidar在imu下的位置，T_imu_lidar
    // Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    // Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    // p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    // p_imu->set_gyr_cov(V3D(p_imu->gyr_cov, p_imu->gyr_cov, p_imu->gyr_cov));
    // p_imu->set_acc_cov(V3D(p_imu->acc_cov, p_imu->acc_cov, p_imu->acc_cov));
    // p_imu->set_gyr_bias_cov(V3D(p_imu->b_gyr_cov, p_imu->b_gyr_cov, p_imu->b_gyr_cov));
    // p_imu->set_acc_bias_cov(V3D(p_imu->b_acc_cov, p_imu->b_acc_cov, p_imu->b_acc_cov));



    scan_count = 0;
    last_timestamp_lidar = 0,       last_timestamp_imu = -1.0;
    timediff_lidar_wrt_imu = 0.0,   time_diff_lidar_to_imu = 0,      lidar_mean_scantime = 0;


    feats_undistort.reset(new PointCloudXYZI());

    kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;

    match_time = 0, solve_time = 0;

    memset(&cur_pose, 0, sizeof(cur_pose));
    memset(&pre_pose, 0, sizeof(pre_pose));

    return true;

}

void MapProcess::SetPcdDir(std::string path_pcd)
{
    my_pcd_dir = root_dir + path_pcd;
}





// 地图相关
void MapProcess::SetPose(state_group init_state)
{
    state_point = kf.get_x();
    state_point.rot = init_state.rot;
    state_point.pos = init_state.pos;

    kf.change_x(state_point);
}
void MapProcess::GetPose(state_group &out_state)
{
    out_state.rot = state_point.rot;
    out_state.pos = state_point.pos;

}
void MapProcess::SetMap(string map_path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    double load_start = omp_get_wtime();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *temp_cloud) == -1) {
        LOG(ERROR) <<"Couldn't read map file from " << map_path;
        abort();
    }
    double load_end = omp_get_wtime();

    LOG(INFO) << "Loaded map  " << temp_cloud->points.size() << " data points from " + map_path ;

    size_t map_size = temp_cloud->size();
    cloud_map->points.resize(map_size);
    for (size_t i = 0; i < map_size; ++i) {
        PointType point;
        point.x = temp_cloud->points[i].x;
        point.y = temp_cloud->points[i].y;
        point.z = temp_cloud->points[i].z;
        point.intensity = temp_cloud->points[i].intensity;
        cloud_map->points[i] = point;
    }
    cloud_map->width = temp_cloud->width;
    cloud_map->height = temp_cloud->height;
    cloud_map->is_dense = temp_cloud->is_dense;


    double build_start = omp_get_wtime();
    if (map_size > 5)
    {
        ikdtree.set_downsample_param(filter_size_map_min);
        ikdtree.Build(cloud_map->points);
        LOG(INFO) << "ikdtree build done, tree size : " << ikdtree.size()<<"   validd num :"<<ikdtree.validnum() ;

    }
    else {
        LOG(ERROR) <<"map points size is too small : " << cloud_map->size();
        abort();
    }
    double build_end = omp_get_wtime();

    LOG(INFO) << "load map cost : " << (load_end-load_start)*1000 << "ms, build map cost : " << (build_end-build_start)*1000 ;


    assert(ikdtree.Root_Node != nullptr);

}
void MapProcess::lasermap_fov_segment()
{
    static bool Localmap_Initialized = false;          // 地图初始化标志

    cub_needrm.clear();             // 清空需要移除的区域
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;

    V3D pos_LiD = pos_lid;          // w系下lidar位置

//    初始化局部地图，以pos_lid为中心，cube_len为长宽高
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }


    float dist_to_map_edge[3][2];       // 各个维度到地图边界的距离,每个维度有正负两个方向
    bool need_move = false;

//    判断是否需要移动，判断阈值为 MOV_THRESHOLD * DET_RANGE
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]); // lidar位置到边界的距离
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) // 如果过于接近，需要移动
            need_move = true;
    }
    if (!need_move)
        return;     //如果不需要，直接返回，不更改局部地图


    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;

//    需要移动的距离 = max( (cube_len - 2*MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9 , DET_RANGE * (MOV_THRESHOLD - 1) )
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));

//    遍历三个维度，
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)        // min的边界，如果过于接近，需要扩充min，消减max
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;     // 待删除的区域
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)   // max的边界，如果过于接近，需要消减min，扩充max
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;     // 待删除的区域
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();     // 收集删除的点
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);     //  删除指定范围内的点
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}
void MapProcess::map_update()
{

}
void MapProcess::map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);


    for (int i = 0; i < feats_down_size; i++)
    {

        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));        //        当前帧降采样后的点云传到world坐标系
        if (!Nearest_Points[i].empty() )       //        如果当前点找到了满足要求的临近点
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;

            PointType  mid_point;       //            找到当前体素中心点坐标
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);     //            判断当前点距离体素中心的距离，
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);   //如果距离最近的点都在体素外，地图中该体素内之前没有点，直接添加当前点，不需要ds
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)  // 遍历找到的临近点，此时可确保所有的临近点均落在当前体素内，如果该体素内的已有点距离体素中心的距离 < 当前点距离，不需要添加当前点
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);    // 当前点距离体素中心距离里更近，添加当前点到体素中， TODO:怎么标记那些距离中心并不是最近的点？在Add_Points中
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);  // 第一帧没有临近点，直接添加当前点
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}
void MapProcess::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
//    当前点与地图点匹配,求残差
//    计算雅克比，也就是点面残差的导数 H

    double match_start = omp_get_wtime();
    laserCloudOri->clear();      // 计算点-面残差时，实际用到的满足要求的点坐标，l系
    corr_normvect->clear();      // 计算点-面残差时，实际用到的点对应平面的参数，w系


#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)       //  遍历所有的特征点
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

//        根据前向传播估计的位姿x_，将lidar坐标系点云转达w系
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];  // 取地址，将会更改Nearest_Points这个vector

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)  pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);  // 点到平面的距离
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points, effective num is " << effct_feat_num;
        return;
    }

    double match_time_once = omp_get_wtime() - match_start;
    match_time += match_time_once;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
    ekfom_data.h.resize(effct_feat_num);


    // 遍历被选中的特征点，计算测量值和测量雅可比矩阵
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    double solve_time_once = omp_get_wtime() - solve_start_;
    solve_time += solve_time_once;

//    LOG(INFO) << " match time : " <<match_time_once*1000;
//    LOG(INFO) << " solve time : " <<solve_time_once*1000;

}

// 小函数
template <typename T>
void MapProcess::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}
void MapProcess::pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
void MapProcess::pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
void MapProcess::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
void MapProcess::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}
void MapProcess::points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);     // 获得移除过的点云，并将相应buffer清空
     for (int i = 0; i < points_history.size(); i++)
         _featsArray->push_back(points_history[i]);
}
void MapProcess::ShowMap(PointCloudXYZI::Ptr laserCloud_world_map, PointCloudXYZI::Ptr laserCloud_world_cur){

    if (vis == NULL)
        vis = new pcl::visualization::PCLVisualizer("local map");

    pcl::PointCloud<pcl::PointXYZI>::Ptr show_cloud_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr show_cloud_cur(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI  pp;

    show_cloud_map->points.reserve(laserCloud_world_map->size());
    show_cloud_cur->points.reserve(laserCloud_world_cur->size());
    for( int ipt=0; ipt<laserCloud_world_map->size(); ipt++ ) {
        pp.x = laserCloud_world_map->points[ipt].x;
        pp.y = laserCloud_world_map->points[ipt].y;
        pp.z = laserCloud_world_map->points[ipt].z;
        pp.intensity = laserCloud_world_map->points[ipt].intensity;
        show_cloud_map->points.push_back(pp);
    }
    for( int ipt=0; ipt<laserCloud_world_cur->size(); ipt++ ) {
        pp.x = laserCloud_world_cur->points[ipt].x;
        pp.y = laserCloud_world_cur->points[ipt].y;
        pp.z = laserCloud_world_cur->points[ipt].z;
        pp.intensity = laserCloud_world_cur->points[ipt].intensity;
        show_cloud_cur->points.push_back(pp);
    }


    vis->removeAllPointClouds();
    vis->addCoordinateSystem(10.0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor_map(show_cloud_map, "intensity"); // 按照 intensity 强度字段进行渲染
    vis->addPointCloud<pcl::PointXYZI>(show_cloud_map, fildColor_map, "map");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor_cur(show_cloud_cur, "z"); // 按照 intensity 强度字段进行渲染
    vis->addPointCloud<pcl::PointXYZI>(show_cloud_cur, fildColor_cur, "cur");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cur");

    vis->spinOnce(10);
}
void MapProcess::GetWholeMap(PointCloudXYZI::Ptr cloud_map)
{
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    double t3 = omp_get_wtime();

    PointCloudXYZI::Ptr laserCloud_world_map(new PointCloudXYZI);
    laserCloud_world_map->clear();
    laserCloud_world_map->resize(ikdtree.PCL_Storage.size());
    laserCloud_world_map->points = ikdtree.PCL_Storage;

    cloud_map->clear();
    cloud_map->resize(laserCloud_world_map->size() + _featsArray->size());
    *cloud_map = *laserCloud_world_map + *_featsArray;

}


void MapProcess::UpdateShowBuff1(PointCloudXYZI::Ptr laserCloud_world_map, PointCloudXYZI::Ptr laserCloud_world_cur)
{

    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int points_num = laserCloudFullRes->points.size();

    double t1 = omp_get_wtime();

    PointVector().swap(ikdtree.PCL_Storage);
    double t2 = omp_get_wtime();

    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    double t3 = omp_get_wtime();

    laserCloud_world_map->clear();
    laserCloud_world_map->resize(ikdtree.PCL_Storage.size());
    laserCloud_world_map->points = ikdtree.PCL_Storage;

    double t4 = omp_get_wtime();

    laserCloud_world_cur->clear();
    laserCloud_world_cur->reserve(points_num);
    for (int i = 0; i < points_num; i++)
    {
        PointType po, pi;
        pi = laserCloudFullRes->points[i];
        V3D p_body(pi.x, pi.y, pi.z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
        laserCloud_world_cur->emplace_back(po);
    }

    cur_pose.x = state_point.pos(0);
    cur_pose.y = state_point.pos(1);
    cur_pose.z = state_point.pos(2);

    double t5 = omp_get_wtime();

    std::cout << " 1 : " << (t2 - t1)*1000 << "\n";
    std::cout << " 1 : " << (t3 - t2)*1000 << "\n";
    std::cout << " 1 : " << (t4 - t3)*1000 << "\n";
    std::cout << " 1 : " << (t5 - t4)*1000 << "\n";



}
void MapProcess::UpdateShowBuff2(PointCloudXYZI::Ptr laserCloud_world_map, PointCloudXYZI::Ptr laserCloud_world_cur)
{

    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int points_num = laserCloudFullRes->points.size();

    double t1 = omp_get_wtime();

    laserCloud_world_cur->clear();
    laserCloud_world_cur->reserve(points_num);
    for (int i = 0; i < points_num; i++)
    {
        PointType po, pi;
        pi = laserCloudFullRes->points[i];
        V3D p_body(pi.x, pi.y, pi.z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
        laserCloud_world_cur->emplace_back(po);
    }

    cur_pose.x = state_point.pos(0);
    cur_pose.y = state_point.pos(1);
    cur_pose.z = state_point.pos(2);

    double t5 = omp_get_wtime();


    std::cout << " 1 : " << (t5 - t1)*1000 << "\n";



}
void MapProcess::UpdateShowBuff(PointCloudXYZI::Ptr laserCloud_world_cur, bool &b_update_flag)
{

    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int points_num = laserCloudFullRes->points.size();

    double t1 = omp_get_wtime();

    laserCloud_world_cur->clear();
    laserCloud_world_cur->reserve(points_num);
    for (int i = 0; i < points_num; i++)
    {
        PointType po, pi;
        pi = laserCloudFullRes->points[i];
        V3D p_body(pi.x, pi.y, pi.z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
        laserCloud_world_cur->emplace_back(po);
    }

    cur_pose.x = state_point.pos(0);
    cur_pose.y = state_point.pos(1);
    cur_pose.z = state_point.pos(2);

    double t2 = omp_get_wtime();


    float d = (cur_pose.x - pre_pose.x) * (cur_pose.x - pre_pose.x)
            + (cur_pose.y - pre_pose.y) * (cur_pose.y - pre_pose.y)
            + (cur_pose.z - pre_pose.z) * (cur_pose.z - pre_pose.z);

    if( feats_undistort->size()>0 ) {
        static int pcd_idx = 0;
        string file_lidar = my_pcd_dir + "/" + to_string( pcd_idx++) + ".pcd";
        feats_undistort_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++)
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_undistort_world->points[i]));
    }


    double t3 = omp_get_wtime();


    if( d>0.5 ) {
        b_update_flag = true;
        memcpy(&pre_pose, &cur_pose, sizeof(cur_pose));
    }


    std::cout << " update buffer : " << (t2 - t1)*1000 << "\n";
    std::cout << " save pcd :      " << (t3 - t2)*1000 << std::endl;



}
void MapProcess::UpdateShowBuff(PointCloudXYZI::Ptr laserCloud_world_cur, PointCloudXYZI::Ptr laserCloud_world_save, bool &b_update_flag, int &count_save_pcd_num)
{

//     获取当前帧body点云
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int points_num = laserCloudFullRes->points.size();

    double t1 = omp_get_wtime();

//    获取当前帧world点云（降采样后的点云用于vis）
    laserCloud_world_cur->clear();
    laserCloud_world_cur->reserve(points_num);
    for (int i = 0; i < points_num; i++)
    {
        PointType po, pi;
        pi = laserCloudFullRes->points[i];
        V3D p_body(pi.x, pi.y, pi.z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
        laserCloud_world_cur->emplace_back(po);
    }


//    获取当前帧位姿
    cur_pose.x = state_point.pos(0);
    cur_pose.y = state_point.pos(1);
    cur_pose.z = state_point.pos(2);

    double t2 = omp_get_wtime();


    float d = (cur_pose.x - pre_pose.x) * (cur_pose.x - pre_pose.x)
            + (cur_pose.y - pre_pose.y) * (cur_pose.y - pre_pose.y)
            + (cur_pose.z - pre_pose.z) * (cur_pose.z - pre_pose.z);


//    存储当前帧world点云
//    if( laserCloud_world_cur->size()>0 ) {
//        static int pcd_idx = 0;
//        string file_lidar = my_pcd_dir + "/" + to_string( pcd_idx++) + ".pcd";
//        pcl::io::savePCDFileBinary(file_lidar, *laserCloud_world_cur);
//    }

    if( feats_undistort->size()>0 ) {
        static int pcd_idx = 0;
        string file_lidar = my_pcd_dir + "/" + to_string( pcd_idx++) + ".pcd";
        feats_undistort_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++)
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_undistort_world->points[i]));
        pcl::io::savePCDFileBinary(file_lidar, *feats_undistort_world);
    }


    double t3 = omp_get_wtime();


    if( d>0.5 ) {
        b_update_flag = true;
        memcpy(&pre_pose, &cur_pose, sizeof(cur_pose));

        *laserCloud_world_save = *laserCloud_world_save + *laserCloud_world_cur;
        count_save_pcd_num++;

        double t4 = omp_get_wtime();
        std::cout << " accumed key frame !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << "\n";
        std::cout << " accumed key frame time :   " << (t4 - t3)*1000 << "\n";
        std::cout << " accumed key frame size :   " << laserCloud_world_save->size() << std::endl;

    }


    std::cout << " update buffer : " << (t2 - t1)*1000 << "\n";
    std::cout << " save pcd :      " << (t3 - t2)*1000 << std::endl;



}


bool MapProcess::Process(PointCloudXYZI::Ptr cloud_frame)
{


    double start = omp_get_wtime();
    double t0, t1, t2, t3, t4, t5, match_start, solve_start;
    double match_time = 0, solve_time = 0, solve_const_H_time = 0, kdtree_search_time = 0, svd_time = 0;
    match_time = 0, solve_time = 0, solve_const_H_time = 0, kdtree_search_time = 0.0, svd_time = 0;


    state_point = kf.get_x();


    feats_undistort = cloud_frame;
    if (feats_undistort->empty() || (feats_undistort == NULL))
    {
        ROS_WARN("No point, skip this scan!\n");
        return false;
    }



// step 2.   根据当前位置，移动地图边界，增加新的area，删除新边界外的点云
    // lasermap_fov_segment();
    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);
    feats_down_size = feats_down_body->points.size();

    LOG(INFO) << "points size less    : " <<feats_undistort->size();
    LOG(INFO) << "points size less ds : " <<feats_down_body->size();

    if (feats_down_size < 5)    //        如果点云个数太少，跳过该帧
    {
        ROS_WARN("No point, skip this scan!\n");
        return false;
    }


    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);

    for (int i = 0; i < feats_down_size; i++)
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));  



    Nearest_Points.resize(feats_down_size); //        存储近邻点的vector

    double solve_H_time = 0;
// step 3.   当前帧点云与地图匹配，迭代更新状态 kf.x_ 和协方差矩阵 kf.P_
    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    state_point = kf.get_x();
    auto P = kf.get_P();

    feats_undistort_world->resize(feats_undistort->size());
    for (int i = 0; i < feats_undistort->size(); i++)
        pointBodyToWorld(&(feats_undistort->points[i]), &(feats_undistort_world->points[i]));


    int tree_valid_num = ikdtree.validnum();
    int tree_all_num = ikdtree.size();

    double end =omp_get_wtime();
{
//        std::cout << "t1 cost time : " << (t1 - start)*1000 << "\n";
//        std::cout << "t2 cost time : " << (t2 - start)*1000 << "\n";
//        std::cout << "t3 cost time : " << (t_update_end - start)*1000 << "\n";
//        std::cout << "t4 cost time : " << (t3 - start)*1000 << "\n";
//        std::cout << "t5 cost time : " << (t5 - start)*1000 << "\n";
//        std::cout<<"R: " <<state_point.offset_R_L_I<<std::endl;
//        std::cout<<"t: " <<state_point.offset_T_L_I<<std::endl;

}

    LOG(INFO) <<"effective points: "<<effct_feat_num<<std::endl;
    // LOG(INFO) << "ikdtree size: " << tree_all_num <<"  valid num : " << tree_valid_num << std::endl;
    LOG(INFO) << "map process cost time : " << (end - start)*1000 ;

    return true;


}
