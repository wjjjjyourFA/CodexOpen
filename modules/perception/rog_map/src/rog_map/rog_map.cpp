/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#include "rog_map/rog_map.h"


using namespace rog_map;

ROGMap::ROGMap(const ros::NodeHandle& nh) :nh_(nh) {

    cfg_ = rog_map::Config(nh);
    initProbMap();

    map_info_log_file_.open(DEBUG_FILE_DIR("rm_info_log.csv"), std::ios::out | std::ios::trunc);
    time_log_file_.open(DEBUG_FILE_DIR("rm_performance_log.csv"), std::ios::out | std::ios::trunc);

    vm_.vizcfg.use_body_center = true;
    vm_.vizcfg.box_min = -cfg_.visualization_range / 2;
    vm_.vizcfg.box_max = cfg_.visualization_range / 2;

    vm_.vizcfg.callback_func = boost::bind(&ROGMap::VizCfgCallback, this, _1, _2);
    vm_.vizcfg.vizcfgserver.setCallback(vm_.vizcfg.callback_func);

    robot_state_.p = cfg_.fix_map_origin;

    if (cfg_.map_sliding_en) {
        mapSliding(Vec3f(0, 0, 0));
        inf_map_->mapSliding(Vec3f(0, 0, 0));
    }
    else {
        /// if disable map sliding, fix map origin to (0,0,0)
        /// update the local map bound as
        local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
        local_map_bound_max_d_ = cfg_.half_map_size_d + cfg_.fix_map_origin;
        mapSliding(cfg_.fix_map_origin);
        inf_map_->mapSliding(cfg_.fix_map_origin);
        vm_.vizcfg.box_min += cfg_.fix_map_origin;
        vm_.vizcfg.box_max += cfg_.fix_map_origin;
    }

    /// Initialize visualization module
    if (cfg_.visualization_en) {
        vm_.occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/occ", 1);
        vm_.unknown_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/unk", 1);
        vm_.occ_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_occ", 1);
        vm_.unknown_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_unk", 1);
        vm_.pc_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/pc", 1);
        vm_.terr_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/terr", 1);

        if (cfg_.frontier_extraction_en) {
            vm_.frontier_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/frontier", 1);
        }

        if (cfg_.esdf_en) {
            vm_.esdf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf", 1);
            vm_.esdf_neg_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf/neg", 1);
            vm_.esdf_occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf/occ", 1);
        }

        if (cfg_.viz_time_rate > 0) {
            vm_.viz_timer = nh_.createTimer(ros::Duration(1.0 / cfg_.viz_time_rate), &ROGMap::vizCallback,
                                            this);
        }
    }
    vm_.mkr_arr_pub = nh_.advertise<visualization_msgs::MarkerArray>("rog_map/map_bound", 1);

    if (cfg_.ros_callback_en) {
        if(cfg_.mode_this == 1){
            rc_.cloud_odom_sub = nh_.subscribe("/location_meta", 1, &ROGMap::cloudOdomCallbackUV, this);
        }else if(cfg_.mode_this == 2){
            rc_.odom_sub = nh_.subscribe(cfg_.odom_topic, 1, &ROGMap::odomCallback, this);
            rc_.cloud_sub = nh_.subscribe(cfg_.cloud_topic, 1, &ROGMap::cloudCallback, this);
            rc_.update_timer = nh_.createTimer(ros::Duration(0.001), &ROGMap::updateCallback, this);
        }else{
            rc_.meta_sub = nh_.subscribe("/PGP_DOR_meta", 1, &ROGMap::meta_cbk, this);
        }
    }

    writeMapInfoToLog(map_info_log_file_);
    map_info_log_file_.close();
    for (int i = 0; i < time_consuming_name_.size(); i++) {
        time_log_file_ << time_consuming_name_[i];
        if (i != time_consuming_name_.size() - 1) {
            time_log_file_ << ", ";
        }
    }
    time_log_file_ << endl;


    if (cfg_.load_pcd_en) {
        string pcd_path = cfg_.pcd_name;
        PointCloud::Ptr pcd_map(new PointCloud);
        if (pcl::io::loadPCDFile(pcd_path, *pcd_map) == -1) {
            cout << RED << "Load pcd file failed!" << RESET << endl;
            exit(-1);
        }
        Pose cur_pose;
        cur_pose.first = Vec3f(0, 0, 0);
        updateOccPointCloud(*pcd_map);
        esdf_map_->updateESDF3D(robot_state_.p);
        cout << BLUE << " -- [ROGMap]Load pcd file success with " << pcd_map->size() << " pts." << RESET << endl;
        map_empty_ = false;
    }
}

bool ROGMap::isLineFree(const rog_map::Vec3f& start_pt, const rog_map::Vec3f& end_pt,
                        const bool& use_inf_map, const bool& use_unk_as_occ) const {
    if(start_pt.array().isNaN().any() || end_pt.array().isNaN().any() ) {
        cout<<RED<<" -- [ROGMap] Call isLineFree with NaN in start or end pt, return false."<<RESET<<endl;
        return false;
    }
    raycaster::RayCaster raycaster;
    if (use_inf_map) {
        raycaster.setResolution(cfg_.inflation_resolution);
    }
    else {
        raycaster.setResolution(cfg_.resolution);
    }
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    while (raycaster.step(ray_pt)) {
        if (!use_unk_as_occ) {
            // allow both unk and free
            if (use_inf_map) {
                if (isOccupiedInflate(ray_pt)) {
                    return false;
                }
            }
            else {
                if (isOccupied(ray_pt)) {
                    return false;
                }
            }
        }
        else {
            // only allow known free
            if (use_inf_map) {
                if ((isUnknownInflate(ray_pt) || isOccupiedInflate(ray_pt)))
                    return false;
            }
            else {
                if (!isKnownFree(ray_pt)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    while (raycaster.step(ray_pt)) {
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) {
                return false;
            }
        }
        else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto& nei : neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, Vec3f& free_local_goal, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    free_local_goal = start_pt;
    while (raycaster.step(ray_pt)) {
        free_local_goal = ray_pt;
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) {
                return false;
            }
        }
        else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto& nei : neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) {
                    return false;
                }
            }
        }
    }
    free_local_goal = end_pt;
    return true;
}

void ROGMap::updateMap(const PointCloud& cloud, const Pose& pose) {
    TimeConsuming ssss("sss", false);
    if (cfg_.ros_callback_en) {
        std::cout << RED << "ROS callback is enabled, can not insert map from updateMap API." << RESET
            << std::endl;
        return;
    }

    if (cloud.empty()) {
        static int local_cnt = 0;
        if (local_cnt++ > 100) {
            cout << YELLOW << "No cloud input, please check the input topic." << RESET << endl;
            local_cnt = 0;
        }
        return;
    }

    updateRobotState(pose);
    updateProbMap(cloud, pose);

    writeTimeConsumingToLog(time_log_file_);
}

RobotState ROGMap::getRobotState() const {
    return robot_state_;
}

void ROGMap::updateRobotState(const Pose& pose) {
    robot_state_.p = pose.first;
    robot_state_.q = pose.second;
    robot_state_.rcv_time = ros::Time::now().toSec();
    robot_state_.rcv = true;
    robot_state_.yaw = get_yaw_from_quaternion<double>(pose.second);
    updateLocalBox(pose.first);
}


void ROGMap::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    updateRobotState(std::make_pair(
        Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
              -odom_msg->pose.pose.position.z),
        Quatf(odom_msg->pose.pose.orientation.w * 3.14159265358979323846 / 180, odom_msg->pose.pose.orientation.x * 3.14159265358979323846 / 180,
              odom_msg->pose.pose.orientation.y * 3.14159265358979323846 / 180, odom_msg->pose.pose.orientation.z * 3.14159265358979323846 / 180)));


    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
    br_map_ego.sendTransform(transformStamped);
}

void ROGMap::odomCallbackVU(const nav_msgs::OdometryConstPtr& odom_msg) {
    updateRobotState(std::make_pair(
        Vec3f(-odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.x,
              odom_msg->pose.pose.position.z),
        Quatf(odom_msg->pose.pose.orientation.w * 3.14159265358979323846 / 180, odom_msg->pose.pose.orientation.x * 3.14159265358979323846 / 180,
              odom_msg->pose.pose.orientation.y * 3.14159265358979323846 / 180, odom_msg->pose.pose.orientation.z * 3.14159265358979323846 / 180)));


    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = -odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x * 3.14159265358979323846 / 180;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y * 3.14159265358979323846 / 180;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z * 3.14159265358979323846 / 180;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w * 3.14159265358979323846 / 180;
    br_map_ego.sendTransform(transformStamped);
}

void ROGMap::cloudOdomCallback(const self_state::PointCloudOdometryConstPtr& cloud_odo_msg) {
    // 获取 PointCloud2 和 Odometry 消息
    const sensor_msgs::PointCloud2ConstPtr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>(cloud_odo_msg->point_cloud);
    const nav_msgs::OdometryConstPtr odom_msg = boost::make_shared<nav_msgs::Odometry>(cloud_odo_msg->odometry);


    // 处理 Odometry 数据
    updateRobotState(std::make_pair(
        Vec3f(-odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.x,
              odom_msg->pose.pose.position.z),
        Quatf(odom_msg->pose.pose.orientation.w * 3.14159265358979323846 / 180, odom_msg->pose.pose.orientation.x * 3.14159265358979323846 / 180,
              odom_msg->pose.pose.orientation.y * 3.14159265358979323846 / 180, odom_msg->pose.pose.orientation.z * 3.14159265358979323846 / 180)));


    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = -odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x * 3.14159265358979323846 / 180;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y * 3.14159265358979323846 / 180;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z * 3.14159265358979323846 / 180;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w * 3.14159265358979323846 / 180;
    br_map_ego.sendTransform(transformStamped);

    // 处理 PointCloud2 数据
    if (!robot_state_.rcv) {
        return;
    }
    double cbk_t = ros::Time::now().toSec();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        return;
    }

    PointCloud temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();

    // 处理更新逻辑
    if (map_empty_) {
        static double last_print_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
            std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET << std::endl;
            last_print_t = cur_t;
        }
        return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
        return;
    }
    else if (rc_.unfinished_frame_cnt > 1) {
        std::cout << RED <<
            " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
            << std::endl;
    }

    static Pose temp_pose;
    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    // 目标存储变换后的点云
    pcl::PointCloud<pcl::PointXYZINormal> rotated_pc, world_pc;

    //////////////////////////////////////

    // 转换为欧拉角
    double roll, pitch, yaw;
    tf::Quaternion tf_quat;
    tf_quat.setX(odom_msg->pose.pose.orientation.x * 3.14159265358979323846 / 180);
    tf_quat.setY(odom_msg->pose.pose.orientation.y * 3.14159265358979323846 / 180);
    tf_quat.setZ(odom_msg->pose.pose.orientation.z * 3.14159265358979323846 / 180);
    tf_quat.setW(odom_msg->pose.pose.orientation.w * 3.14159265358979323846 / 180);

    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    Eigen::Matrix3d Rx,Ry,Rz,R_w_l;
    double crz = cos(yaw);
    double srz = sin(yaw);
    double crx = cos(-pitch);
    double srx = sin(-pitch);
    double cry = cos(roll);
    double sry = sin(roll);
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_l = Rz*Rx*Ry;

    //////////////////////////////////////

    // 遍历原始点云并进行变换
    for (const auto &point : temp_pc)
    {
        pcl::PointXYZINormal transformed_point = point;

        Eigen::Vector3d rotated_point = R_w_l * Eigen::Vector3d(-point.y, point.x, point.z);

        // 更新坐标
        transformed_point.x = rotated_point.x() + temp_pose.first.x();
        transformed_point.y = rotated_point.y() + temp_pose.first.y();
        transformed_point.z = rotated_point.z() + temp_pose.first.z();

        world_pc.push_back(transformed_point);
    }

    rc_.updete_lock.lock();
    rc_.pc = world_pc;
    rc_.updete_lock.unlock();

    updateProbMap(world_pc, temp_pose);

    // 创建 ROS 点云消息
    sensor_msgs::PointCloud2 output_msg;

    // 转换 PCL 点云到 ROS 消息
    pcl::toROSMsg(world_pc, output_msg);

    // 设置消息头部信息
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "camera_init"; // 需要替换为你的坐标系，例如 "odom" 或 "base_link"

    // 通过 ROS 话题发布
    vm_.pc_pub.publish(output_msg);

    writeTimeConsumingToLog(time_log_file_);
}

void ROGMap::cloudOdomCallbackUV(const Metascan::metascanConstPtr& cloud_odo_msg) {
    // 获取 PointCloud2 和 Odometry 消息
    const sensor_msgs::PointCloud2ConstPtr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>(cloud_odo_msg->cloud);
    // const nav_msgs::OdometryConstPtr odom_msg = boost::make_shared<nav_msgs::Odometry>(cloud_odo_msg->odometry);

    

    // 处理 Odometry 数据
    updateRobotState(std::make_pair(
        Vec3f(cloud_odo_msg->pose[0], cloud_odo_msg->pose[1],
              cloud_odo_msg->pose[2]),
        Quatf(cloud_odo_msg->pose[3], cloud_odo_msg->pose[4],
              cloud_odo_msg->pose[5], 0.0)));


    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = cloud_odo_msg->pose[0];
    transformStamped.transform.translation.y = cloud_odo_msg->pose[1];
    transformStamped.transform.translation.z = cloud_odo_msg->pose[2];
    transformStamped.transform.rotation.x = cloud_odo_msg->pose[3];
    transformStamped.transform.rotation.y = cloud_odo_msg->pose[4];
    transformStamped.transform.rotation.z = cloud_odo_msg->pose[5];
    transformStamped.transform.rotation.w = 0.0;
    br_map_ego.sendTransform(transformStamped);

    // 处理 PointCloud2 数据
    if (!robot_state_.rcv) {
        return;
    }
    double cbk_t = ros::Time::now().toSec();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        return;
    }

    PointCloud temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();

    // 处理更新逻辑
    if (map_empty_) {
        static double last_print_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
            std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET << std::endl;
            last_print_t = cur_t;
        }
        return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
        return;
    }
    else if (rc_.unfinished_frame_cnt > 1) {
        std::cout << RED <<
            " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
            << std::endl;
    }

    static Pose temp_pose;
    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    // 目标存储变换后的点云
    pcl::PointCloud<pcl::PointXYZINormal> rotated_pc, world_pc;

    //////////////////////////////////////

    // 转换为欧拉角
    // double roll, pitch, yaw;
    // tf::Quaternion tf_quat;
    // tf_quat.setX(odom_msg->pose.pose.orientation.x * 3.14159265358979323846 / 180);
    // tf_quat.setY(odom_msg->pose.pose.orientation.y * 3.14159265358979323846 / 180);
    // tf_quat.setZ(odom_msg->pose.pose.orientation.z * 3.14159265358979323846 / 180);
    // tf_quat.setW(odom_msg->pose.pose.orientation.w * 3.14159265358979323846 / 180);

    // tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    // Eigen::Matrix3d Rx,Ry,Rz,R_w_l;
    // double crz = cos(yaw);
    // double srz = sin(yaw);
    // double crx = cos(-pitch);
    // double srx = sin(-pitch);
    // double cry = cos(roll);
    // double sry = sin(roll);
    // Ry<<cry, 0, sry,
    //         0 ,1, 0,
    //         -sry, 0, cry;
    // Rx<<1, 0, 0,
    //         0, crx, -srx,
    //         0, srx, crx;
    // Rz<<crz, -srz, 0,
    //         srz, crz, 0,
    //         0 , 0, 1;
    // R_w_l = Rz*Rx*Ry;

    Eigen::Matrix3d Rx,Ry,Rz,R_w_l;
    double crz = cos(cloud_odo_msg->pose[5]);
    double srz = sin(cloud_odo_msg->pose[5]);
    double cry = cos(cloud_odo_msg->pose[4]);
    double sry = sin(cloud_odo_msg->pose[4]);
    double crx = cos(cloud_odo_msg->pose[3]);
    double srx = sin(cloud_odo_msg->pose[3]);
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_l = Rz*Ry*Rx; // R_azimuth * R_pitch * R_roll

    //////////////////////////////////////

    // 遍历原始点云并进行变换
    for (const auto &point : temp_pc)
    {
        pcl::PointXYZINormal transformed_point = point;

        Eigen::Vector3d rotated_point = R_w_l * Eigen::Vector3d(point.x, point.y, point.z);

        // 更新坐标
        transformed_point.x = rotated_point.x() + temp_pose.first.x();
        transformed_point.y = rotated_point.y() + temp_pose.first.y();
        transformed_point.z = rotated_point.z() + temp_pose.first.z();

        world_pc.push_back(transformed_point);
    }

    rc_.updete_lock.lock();
    rc_.pc = world_pc;
    rc_.updete_lock.unlock();

    updateProbMap(world_pc, temp_pose);

    // 创建 ROS 点云消息
    sensor_msgs::PointCloud2 output_msg;

    // 转换 PCL 点云到 ROS 消息
    pcl::toROSMsg(world_pc, output_msg);

    // 设置消息头部信息
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "camera_init"; // 需要替换为你的坐标系，例如 "odom" 或 "base_link"

    // 通过 ROS 话题发布
    vm_.pc_pub.publish(output_msg);

    writeTimeConsumingToLog(time_log_file_);
}


void ROGMap::meta_cbk(const Metascan::metascan::ConstPtr &meta_in)
{
    sensor_msgs::PointCloud2::Ptr cldi(new sensor_msgs::PointCloud2(meta_in->cloud));

    updateRobotState(std::make_pair(
        Vec3f(meta_in->pose[0], meta_in->pose[1],
              meta_in->pose[2]),
        Quatf(0.0, 0.0,
              0.0, 1.0)));


    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = meta_in->pose[0];
    transformStamped.transform.translation.y = meta_in->pose[1];
    transformStamped.transform.translation.z = meta_in->pose[2];
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    br_map_ego.sendTransform(transformStamped);

    // 处理 PointCloud2 数据
    if (!robot_state_.rcv) {
        return;
    }
    double cbk_t = ros::Time::now().toSec();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        return;
    }

    PointCloud temp_pc;
    pcl::fromROSMsg(*cldi, temp_pc);
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();

    // 处理更新逻辑
    if (map_empty_) {
        static double last_print_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
            std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET << std::endl;
            last_print_t = cur_t;
        }
        return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
        return;
    }
    else if (rc_.unfinished_frame_cnt > 1) {
        std::cout << RED <<
            " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
            << std::endl;
    }

    static Pose temp_pose;
    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    //////////////////////////////////////

    // 目标存储变换后的点云
    pcl::PointCloud<pcl::PointXYZINormal> rotated_pc, world_pc;

    // 遍历原始点云并进行变换
    for (const auto &point : temp_pc)
    {
        pcl::PointXYZINormal transformed_point = point;

        if(point.intensity < 40){   //terrain
            transformed_point.x = point.x + temp_pose.first.x();
            transformed_point.y = point.y + temp_pose.first.y();
            transformed_point.z = point.z + temp_pose.first.z();
            rotated_pc.push_back(transformed_point);
            continue;
        }else if(point.intensity < 60){     //non-dynamic

        }else{      // dynamic
            continue;
        }

        // 更新坐标
        transformed_point.x = point.x + temp_pose.first.x();
        transformed_point.y = point.y + temp_pose.first.y();
        transformed_point.z = point.z + temp_pose.first.z();

        world_pc.push_back(transformed_point);
    }

    rc_.updete_lock.lock();
    rc_.pc = world_pc;
    rc_.updete_lock.unlock();

    updateProbMap(world_pc, temp_pose);

    // // 创建 ROS 点云消息
    // sensor_msgs::PointCloud2 output_msg;

    // // 转换 PCL 点云到 ROS 消息
    // pcl::toROSMsg(world_pc, output_msg);

    // // 设置消息头部信息
    // output_msg.header.stamp = ros::Time::now();
    // output_msg.header.frame_id = "camera_init"; // 需要替换为你的坐标系，例如 "odom" 或 "base_link"

    // // 通过 ROS 话题发布
    // vm_.pc_pub.publish(output_msg);

    sensor_msgs::PointCloud2 terr_msg;
    pcl::toROSMsg(rotated_pc, terr_msg);

    terr_msg.header.stamp = ros::Time::now();
    terr_msg.header.frame_id = "world"; // 需要替换为你的坐标系，例如 "odom" 或 "base_link"

    
    vm_.terr_pub.publish(terr_msg);

    writeTimeConsumingToLog(time_log_file_);
}

void ROGMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (!robot_state_.rcv) {
        return;
    }
    double cbk_t = ros::Time::now().toSec();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        return;
    }
    PointCloud temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    rc_.updete_lock.lock();
    for (auto &point : temp_pc)
    {
        point.z = point.z * cfg_.nag_or_pos;
    }
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
}

void ROGMap::updateCallback(const ros::TimerEvent& event) {
    if (map_empty_) {
        static double last_print_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
            std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET << std::endl;
            last_print_t = cur_t;
        }
        return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
        return;
    }
    else if (rc_.unfinished_frame_cnt > 1) {
        std::cout << RED <<
            " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
            << std::endl;
    }
    static PointCloud temp_pc;
    static Pose temp_pose;
    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();


    // 4. 传递变换后的点云到 `updateProbMap`
    updateProbMap(temp_pc, temp_pose);

    // 1. 创建 ROS 点云消息
    sensor_msgs::PointCloud2 output_msg;

    // 2. 转换 PCL 点云到 ROS 消息
    pcl::toROSMsg(temp_pc, output_msg);

    // 3. 设置消息头部信息
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "body"; // 需要替换为你的坐标系，例如 "odom" 或 "base_link"

    // 4. 通过 ROS 话题发布
    vm_.pc_pub.publish(output_msg);

    //auto start_time = std::chrono::high_resolution_clock::now();
    //updateProbMap(temp_pc, temp_pose);
    //auto end_time = std::chrono::high_resolution_clock::now();

    // 计算耗时（毫秒）
    //auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    //std::cout << GREEN << " -- [ROG LOG] UpdateProbMap Over, Time cost :" << duration << " ms" << RESET
    //          << std::endl;

    writeTimeConsumingToLog(time_log_file_);
}

void ROGMap::vecEVec3fToPC2(const vec_E<Vec3f>& points, sensor_msgs::PointCloud2& cloud) {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
        pcl_cloud[i].x = static_cast<float>(points[i][0]);
        pcl_cloud[i].y = static_cast<float>(points[i][1]);
        pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
}

void ROGMap::vizCallback(const ros::TimerEvent& event) {
    TimeConsuming ssss("vizCallback", false);

    if (!cfg_.visualization_en) {
        return;
    }
    if (map_empty_) {
        return;
    }
    Vec3f box_min, box_max;

    // if use dynamic reconfigure
    if (cfg_.use_dynamic_reconfigure) {
        box_min = vm_.vizcfg.box_min;
        box_max = vm_.vizcfg.box_max;

        // if use body center
        if (vm_.vizcfg.use_body_center) {
            box_min += robot_state_.p;
            box_max += robot_state_.p;
        }
    }
    else {
        box_max = robot_state_.p + cfg_.visualization_range / 2;
        box_min = robot_state_.p - cfg_.visualization_range / 2;
    }
    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
        cout << RED << " -- [ROGMap] Visualization range is too small." << RESET << endl;
        return;
    }

    if (cfg_.pub_unknown_map_en && vm_.unknown_pub.getNumSubscribers() >= 1) {
        vec_E<Vec3f> unknown_map, inf_unknown_map;
        boxSearch(box_min, box_max, UNKNOWN, unknown_map);
        sensor_msgs::PointCloud2 cloud_msg;
        vecEVec3fToPC2(unknown_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.unknown_pub.publish(cloud_msg);
        if (cfg_.unk_inflation_en && vm_.unknown_inf_pub.getNumSubscribers() >= 1) {
            boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
            vecEVec3fToPC2(inf_unknown_map, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            vm_.unknown_inf_pub.publish(cloud_msg);
        }
    }

    if (cfg_.frontier_extraction_en && vm_.frontier_pub.getNumSubscribers() >= 1) {
        vec_E<Vec3f> frontier_map;
        boxSearch(box_min, box_max, FRONTIER, frontier_map);
        sensor_msgs::PointCloud2 cloud_msg;
        vecEVec3fToPC2(frontier_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.frontier_pub.publish(cloud_msg);
    }

    vec_E<Vec3f> occ_map, inf_occ_map;
    sensor_msgs::PointCloud2 cloud_msg;
    if (vm_.occ_pub.getNumSubscribers() >= 1) {
        boxSearch(box_min, box_max, OCCUPIED, occ_map);
        vecEVec3fToPC2(occ_map, cloud_msg);
        vm_.occ_pub.publish(cloud_msg);
    }

    if (vm_.occ_inf_pub.getNumSubscribers() >= 1) {
        boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
        vecEVec3fToPC2(inf_occ_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.occ_inf_pub.publish(cloud_msg);
    }

    /* visualize ESDF Map*/
    if (cfg_.esdf_en) {
        if (vm_.esdf_pub.getNumSubscribers() >= 1) {
            esdf_map_->getPositiveESDFPC2(box_min, box_max, robot_state_.p.z() - 0.5, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            vm_.esdf_pub.publish(cloud_msg);
        }

        if (vm_.esdf_neg_pub.getNumSubscribers() >= 1) {
            esdf_map_->getNegativeESDFPC2(box_min, box_max, robot_state_.p.z() - 0.5, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            vm_.esdf_neg_pub.publish(cloud_msg);
        }

#ifdef ESDF_MAP_DEBUG
        esdf_map_->getESDFOccPC2(box_min, box_max,cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.esdf_occ_pub.publish(cloud_msg);
#endif
    }


    /* Publish visualization range */
    vm_.mkr_arr.markers.clear();
    visualizeBoundingBox(vm_.mkr_arr, box_min, box_max, "Visualization Range", Color::Purple());
    visualizeText(vm_.mkr_arr, "Visualization Range Text", "Visualization Range", box_max + Vec3f(0, 0, 0.5),
                  Color::Purple(), 0.6, 0);

    /* Publish local map range */
    Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
    boundBoxByLocalMap(local_map_min, local_map_max);
    visualizeBoundingBox(vm_.mkr_arr, local_map_min, local_map_max, "Local Map Range",
                         Color::Orange());
    visualizeText(vm_.mkr_arr, "Local Map Range Text", "Local Map Range", local_map_max + Vec3f(0, 0, 1.0),
                  Color::Orange(),
                  0.6, 0);

    /* Publish Ray-casting range */
    visualizeBoundingBox(vm_.mkr_arr, raycast_data_.cache_box_min, raycast_data_.cache_box_max,
                         "Updating Range",
                         Color::Green());
    visualizeText(vm_.mkr_arr, "Updating Range Text", "Updating Range",
                  raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
                  Color::Green(), 0.6, 0);

    /* Publish Local map origin */
    visualizePoint(vm_.mkr_arr, local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2, 0);

    if (cfg_.esdf_en) {
        Vec3f esdf_box_max, esdf_box_min;
        esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
        visualizeText(vm_.mkr_arr, "ESDF Map Text", "ESDF Map", esdf_box_max + Vec3f(0, 0, 1.0),
                      Color::Blue(),
                      0.6, 0);
        visualizeBoundingBox(vm_.mkr_arr, esdf_box_min, esdf_box_max, "ESDF Updating Range",
                             Color::Blue());
    }

    vm_.mkr_arr_pub.publish(vm_.mkr_arr);

}

void ROGMap::VizCfgCallback(robot_plan_expv2_ros1::VizConfig& config, uint32_t level) {
    vm_.vizcfg.use_body_center = config.use_body_center;
    vm_.vizcfg.box_min.x() = config.x_lower_bound;
    vm_.vizcfg.box_min.y() = config.y_lower_bound;
    vm_.vizcfg.box_min.z() = config.z_lower_bound;
    vm_.vizcfg.box_max.x() = config.x_upper_bound;
    vm_.vizcfg.box_max.y() = config.y_upper_bound;
    vm_.vizcfg.box_max.z() = config.z_upper_bound;
}
