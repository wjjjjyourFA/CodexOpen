/**
 * @file LOcalization.cpp
 * @author Bokai ()
 * @brief 
 * @version 0.1
 * @date 2024-04-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "localization_ros1.h"

#include <cmath>

bool Localization::b_flg_exit_(false);   // 静态成员变量初始化
std::condition_variable Localization::sig_buffer_;

Localization::Localization(ros::NodeHandle p_node,
                           const std::string& runtime_config,
                           const std::string& map_path,
                           const std::string& log_path)
    : p_node_(p_node),
      config_yaml_(runtime_config),
      map_path_override_(map_path),
      log_path_override_(log_path)
{
    Init();

    LOG(INFO) << "Mid struct func .";

    sub_lidar_ = p_pre_->lidar_type == AVIA ? p_node_.subscribe(topic_lidar_livox_, lidar_queue_size_, &Localization::livox_pcl_cbk, this):
                                              p_node_.subscribe(topic_lidar_, lidar_queue_size_, &Localization::standard_pcl_cbk, this);

    sub_imu_ = p_pre_->lidar_type == AVIA ? p_node_.subscribe(topic_imu_livox_, imu_queue_size_, &Localization::livox_imu_cbk, this):
                                            p_node_.subscribe(topic_imu_, imu_queue_size_, &Localization::raw_imu_cbk, this);


    sub_rviz_pose_ = p_node_.subscribe(topic_initial_pose_, initial_pose_queue_size_, &Localization::rvizPose_cbk, this);


    pub_map_   =  p_node_.advertise<sensor_msgs::PointCloud2>(topic_map_, map_queue_size_, map_latched_);
    pub_path_  =  p_node_.advertise<nav_msgs::Path>(topic_path_, path_queue_size_);
    pub_lio_odom_ = p_node_.advertise<nav_msgs::Odometry>(topic_lio_odom_, odometry_queue_size_);
    pub_lio_path_ = p_node_.advertise<nav_msgs::Path>(topic_lio_path_, lio_path_queue_size_);
    pub_odometry = p_node_.advertise<nav_msgs::Odometry> (topic_state_estimation_, odometry_queue_size_);
    pub_frame_ =   p_node_.advertise<sensor_msgs::PointCloud2>(topic_registered_scan_, registered_scan_queue_size_);


    // pub_frame_ =  p_node_.advertise<sensor_msgs::PointCloud2>("/world_state/cloud_frame", 10);
    // sub_global_pose_ = p_node_.subscribe(topic_global_pose_, 5, &Localization::globalPose_cbk, this);
    // sub_local_pose_  = p_node_.subscribe(topic_local_pose_ , 5, &Localization::localPose_cbk, this);
    // pub_result_mat = p_node.advertise<sensor_msgs::CompressedImage>("/world_state/result_mat/compressed", 5);
    // pub_lidarPose_ = p_node_.advertise<self_state::LidarGlobalPose>(topic_lidar_pose_, 10);
    // pub_fakeGPose  = p_node.advertise<self_state::GlobalPose>(topic_fakeGPose, 5);

    ros::Duration(3.0).sleep();


    path_vec_.header.stamp = ros::Time::now();
    path_vec_.header.frame_id = frame_map_;
    lio_path_vec_.header.stamp = path_vec_.header.stamp;
    lio_path_vec_.header.frame_id = frame_map_;
    LOG(INFO) << "End struct func .";
}

Localization::~Localization()
{

}

// static void Localization::SigHandle(int sig)
// {
//     b_flg_exit_ = true;
//     ROS_WARN("catch sig %d", sig);
//     sig_buffer_.notify_all();
// }


void Localization::ReadTopic()
{
    p_node_.param<std::string>("topics/lidar_standard", topic_lidar_, "/rslidar_points/main");
    p_node_.param<std::string>("topics/imu_standard", topic_imu_, "SensorMsgsIMU");
    p_node_.param<std::string>("topics/lidar_livox", topic_lidar_livox_, "/livox/lidar");
    p_node_.param<std::string>("topics/imu_livox", topic_imu_livox_, "/imu/data");
    p_node_.param<std::string>("topics/global_pose", topic_global_pose_, "/self_state/GlobalPose_ugv");
    p_node_.param<std::string>("topics/local_pose", topic_local_pose_, "/self_state/LocalPose");
    p_node_.param<std::string>("topics/lidar_global_pose", topic_lidar_pose_, "/self_state/LidarGlobalPose");
    p_node_.param<std::string>("topics/initial_pose", topic_initial_pose_, "/initialpose");
    p_node_.param<std::string>("topics/map", topic_map_, "/world_state/cloud_map");
    p_node_.param<std::string>("topics/path", topic_path_, "/world_state/path");
    p_node_.param<std::string>("topics/lio_odom", topic_lio_odom_, "/lio/odom");
    p_node_.param<std::string>("topics/lio_path", topic_lio_path_, "/lio/path");
    p_node_.param<std::string>("topics/state_estimation", topic_state_estimation_, "/state_estimation");
    p_node_.param<std::string>("topics/registered_scan", topic_registered_scan_, "/registered_scan");
    p_node_.param<std::string>("frames/map", frame_map_, "map");
    p_node_.param<std::string>("frames/world", frame_world_, "world");
    p_node_.param<std::string>("frames/sensor", frame_sensor_, "sensor");
    p_node_.param<std::string>("frames/body", frame_body_, "body");
    p_node_.param("queues/lidar", lidar_queue_size_, 5);
    p_node_.param("queues/imu", imu_queue_size_, 200000);
    p_node_.param("queues/initial_pose", initial_pose_queue_size_, 1);
    p_node_.param("queues/map", map_queue_size_, 10);
    p_node_.param("queues/path", path_queue_size_, 5);
    p_node_.param("queues/odometry", odometry_queue_size_, 100);
    p_node_.param("queues/lio_path", lio_path_queue_size_, 1);
    p_node_.param("queues/registered_scan", registered_scan_queue_size_, 10);
    p_node_.param("transport/map_latched", map_latched_, false);

    cout<<"\033[1;32m topic info : \033[0m"<<endl;
    cout<<"topic_lidar : "      <<topic_lidar_<<endl;
    cout<<"topic_imu   : "      <<topic_imu_<<endl;
    cout<<"topic_global_pose : "<<topic_global_pose_<<endl;
    cout<<"topic_local_pose : " <<topic_local_pose_<<endl;
    cout<<"topic_lidar_pose : " <<topic_lidar_pose_<<endl;

}


void Localization::ReadConfig()
{
    LOG(INFO) << "load yaml from " << config_yaml_;
    YAML::Node yaml;

    try {
        yaml = YAML::LoadFile(config_yaml_);
    
        lidar_type_         = yaml["common"]["lidar_type"].as<int>(RS128);
        b_with_ui_          = yaml["common"]["with_ui"].as<bool>(false);
        b_publish_ros_      = yaml["common"]["publish_ros"].as<bool>(true);

        point_filter_num_   = yaml["common"]["point_filter_num"].as<int>(5);

        init_method_        = yaml["init"]["init_method"].as<int>(1);
        hand_pose_          = yaml["init"]["init_pose"].as<vector<double>>();
        delta_pose_         = yaml["init"]["delta_pose"].as<vector<double>>();
        map_center_         = yaml["init"]["map_center"].as<vector<double>>();
        b_display_init_     = yaml["init"]["b_display_init"].as<bool>(false);
        b_twice_init_       = yaml["init"]["b_twice_init"].as<bool>(true);

        b_save_log_         = yaml["io"]["save_log"].as<bool>(true);
        path_log_           = yaml["io"]["path_log"].as<std::string>("Log/");
        path_load_map_      = yaml["io"]["path_load_map"].as<std::string>("Map/target_map.pcd");
        if (!map_path_override_.empty())
            path_load_map_ = map_path_override_;
        if (!log_path_override_.empty())
            path_log_ = log_path_override_;

        threshold_position_ = yaml["threshold"]["threshold_position"].as<double>(15);
        threshold_rotation_ = yaml["threshold"]["threshold_rotation"].as<double>(10);
        threshold_rotation_ = threshold_rotation_*M_PI/180.0;

        p_pre_->lidar_type       = yaml["common"]["lidar_type"].as<int>(RS128);
        p_pre_->blind            = yaml["preprocess"]["blind"].as<double>(5);
        p_pre_->N_SCANS          = yaml["preprocess"]["scan_line"].as<int>(16);
        p_pre_->time_unit        = yaml["preprocess"]["timestamp_unit"].as<int>(SEC);
        p_pre_->SCAN_RATE        = yaml["preprocess"]["scan_rate"].as<int>(10);
        p_pre_->feature_enabled  = yaml["preprocess"]["feature_extract_enable"].as<bool>(false);

        const double box_x_min = yaml["preprocess"]["x_min"].as<double>(-0.7);
        const double box_x_max = yaml["preprocess"]["x_max"].as<double>(0.7);
        const double box_y_min = yaml["preprocess"]["y_min"].as<double>(-0.4);
        const double box_y_max = yaml["preprocess"]["y_max"].as<double>(0.4);
        const double box_z_min = yaml["preprocess"]["z_min"].as<double>(-0.6);
        const double box_z_max = yaml["preprocess"]["z_max"].as<double>(0.5);
        const double max_range = yaml["preprocess"]["maxrange"].as<double>(15.0);

        if (!std::isfinite(box_x_min) || !std::isfinite(box_x_max) ||
            !std::isfinite(box_y_min) || !std::isfinite(box_y_max) ||
            !std::isfinite(box_z_min) || !std::isfinite(box_z_max) ||
            !std::isfinite(max_range) ||
            box_x_min >= box_x_max || box_y_min >= box_y_max ||
            box_z_min >= box_z_max || max_range <= 0.0)
        {
            LOG(ERROR) << "Invalid preprocess valid-region parameters";
            abort();
        }
        p_pre_->SetValidRegion(box_x_min, box_x_max,
                               box_y_min, box_y_max,
                               box_z_min, box_z_max,
                               max_range);
        p_pre_->point_filter_num = point_filter_num_;
        LOG(INFO) << "Valid region inner box: [" << box_x_min << ", " << box_x_max
                  << "] x [" << box_y_min << ", " << box_y_max
                  << "] x [" << box_z_min << ", " << box_z_max
                  << "], max range: " << max_range << " m";

        p_imu_->gyr_cov          = yaml["imuprocess"]["gyr_cov"].as<double>(0.1);
        p_imu_->acc_cov          = yaml["imuprocess"]["acc_cov"].as<double>(0.1);
        p_imu_->b_gyr_cov        = yaml["imuprocess"]["b_gyr_cov"].as<double>(0.0001);
        p_imu_->b_acc_cov        = yaml["imuprocess"]["b_acc_cov"].as<double>(0.0001);
        p_imu_->extrinT                 = yaml["imuprocess"]["extrinsic_T"].as<std::vector<double>>(std::vector<double>());
        p_imu_->extrinR                 = yaml["imuprocess"]["extrinsic_R"].as<std::vector<double>>(std::vector<double>());

        p_map_->extrinsic_est_en        = yaml["mapprocess"]["extrinsic_est_en"].as<bool>(true);
        p_map_->DET_RANGE               = yaml["mapprocess"]["det_range"].as<float>(300.f);
        p_map_->cube_len                = yaml["mapprocess"]["cube_side_length"].as<double>(200);
        p_map_->filter_size_surf_min    = yaml["mapprocess"]["filter_size_surf"].as<double>(0.5);
        p_map_->filter_size_map_min     = yaml["mapprocess"]["filter_size_map"].as<double>(0.5);
        p_map_->NUM_MAX_ITERATIONS      = yaml["mapprocess"]["max_iteration"].as<int>(4);

        z_offset_           = yaml["offset"]["z_offset"].as<double>(0.0);

    } catch (...) {
        LOG(ERROR) << "Fail to open yaml file " << config_yaml_ << std::endl;
        abort();
    }

    V3D Lidar_T_wrt_IMU = Eigen::Vector3d::Zero();
    M3D Lidar_R_wrt_IMU = Eigen::Matrix3d::Zero();
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(p_imu_->extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(p_imu_->extrinR);
    p_imu_->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu_->set_gyr_cov(V3D(p_imu_->gyr_cov, p_imu_->gyr_cov, p_imu_->gyr_cov));
    p_imu_->set_acc_cov(V3D(p_imu_->acc_cov, p_imu_->acc_cov, p_imu_->acc_cov));
    p_imu_->set_gyr_bias_cov(V3D(p_imu_->b_gyr_cov, p_imu_->b_gyr_cov, p_imu_->b_gyr_cov));
    p_imu_->set_acc_bias_cov(V3D(p_imu_->b_acc_cov, p_imu_->b_acc_cov, p_imu_->b_acc_cov));

    std::cout<<yaml<<std::endl;
    // map_center_[0] += delta_pose_[0];
    // map_center_[1] += delta_pose_[1];
    // map_center_[2] += delta_pose_[2];


}


bool Localization::Init()
{
    p_imu_ = make_shared<ImuProcess>();
    p_pre_ = make_shared<PreProcess>();
    p_map_ = make_shared<MapProcess>();
    
    ReadTopic();
    ReadConfig();

    cloud_frame_.reset(new PointCloudXYZI());
    cloud_frame_less_.reset(new PointCloudXYZI());
    if( b_display_init_ )
        vis.reset(new pcl::visualization::PCLVisualizer("Viewer"));

    string lio_yaml = lidar_type_ == AVIA ? ROOT_DIR + string("config/avia.yaml") :
                                            ROOT_DIR + string("config/rslidar.yaml") ;
    p_map_->InitConfig(lio_yaml);
    p_map_->SetMap(path_load_map_);

//     if( b_with_ui_ ) {
//         p_ui_ = std::make_shared<Viewer>(config_yaml_);
//         if ( !p_ui_->Init() ) {
//             LOG(ERROR) << "failed to init ui thread.";
//             return -1;
//         }
//         render_thread_ = std::thread(&Viewer::Run, ui_.get());
// //        render_thread_.detach();
//     }

    
    LOG(INFO)<< "path log : " << path_log_;


    if( b_save_log_ ) {
        if( !filesystem::exists( path_log_))     filesystem::create_directories( path_log_);

        string file_pose =  path_log_ + "localization_pose.txt";
        string file_time =  path_log_ + "localization_time.txt";

        fout_log_pose_.open(file_pose);
        fout_log_time_.open(file_time);
    }
    fout_error.open("error.txt");
    fout_gnss.open("log/gnss.txt");
    fout_loc.open("log/loc.txt");

    LOG(INFO) << "end init . ";


    // sattle_utiler.Init(config_yaml_);
    // sattle_mat = sattle_utiler.sattle_mat.clone();
    // intensity_mat = cv::Mat::ones(sattle_mat.size(), CV_32F) * std::numeric_limits<float>::lowest();

    return true;

}

ros::Duration time_duration(0.1);
void Localization::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_ori)
{
//    LOG(INFO) << "get pcl msg .";

    double start, end;
    start = omp_get_wtime();

    mtx_buffer_.lock();
    scan_count_ ++;
    
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_ori));
    msg->header.stamp -= time_duration;

    if (msg->header.stamp.toSec() < last_timestamp_lidar_)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer_.clear();
    }
    last_timestamp_lidar_ =  msg->header.stamp.toSec();

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre_->Process(msg, ptr);
    
    lidar_buffer_.push_back(ptr);
    time_buffer_.push_back( msg->header.stamp.toSec());  // 帧头时间

    mtx_buffer_.unlock();
    sig_buffer_.notify_all();

    end = omp_get_wtime();
    // LOG(INFO) << "pre process cost time : " << (end - start)*1000 ;

}
void Localization::livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
//    LOG(INFO) << "get livox msg .";

    mtx_buffer_.lock();
    scan_count_ ++;
    double preprocess_start_time = omp_get_wtime();

    if (msg->header.stamp.toSec() < last_timestamp_lidar_)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer_.clear();
    }
    last_timestamp_lidar_ = msg->header.stamp.toSec();       // 雷达帧头


    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre_->Process(msg, ptr);
    lidar_buffer_.push_back(ptr);
    time_buffer_.push_back(last_timestamp_lidar_);
    LOG(INFO) << "cbk cloud size : " << ptr->size();

    mtx_buffer_.unlock();
    sig_buffer_.notify_all();
}
void Localization::raw_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_raw)
{
//    LOG(INFO) << "get imu msg .";

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_raw));
    imuRawToRos(msg_raw, msg);
    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer_.clear();
    }
    last_timestamp_imu_ = timestamp;
    imu_buffer_.push_back(msg);
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();

}
void Localization::livox_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{

    //  LOG(INFO) << "get livox imu .";
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->angular_velocity.x = msg_in->angular_velocity.x;
    msg->angular_velocity.y = msg_in->angular_velocity.y;
    msg->angular_velocity.z = msg_in->angular_velocity.z;
    msg->linear_acceleration.x = msg_in->linear_acceleration.x;
    msg->linear_acceleration.y = msg_in->linear_acceleration.y;
    msg->linear_acceleration.z = msg_in->linear_acceleration.z;
    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer_.clear();
    }
    last_timestamp_imu_ = timestamp;
    imu_buffer_.push_back(msg);
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();

}
void Localization::globalPose_cbk(const self_state::GlobalPose::ConstPtr &msg_in)
{
    //  LOG(INFO) << "get global pose.";

    double timestamp = msg_in->local_time;

    nav_msgs::Odometry::Ptr gnssMsg(new nav_msgs::Odometry);
    gnssMsg->header.stamp = ros::Time().fromSec(msg_in->local_time/1000.0);
    // gnssMsg->pose.pose.position.x = msg_in->gaussX - map_center_[0];
    // gnssMsg->pose.pose.position.y = msg_in->gaussY - map_center_[1];
    // gnssMsg->pose.pose.position.z = msg_in->height - map_center_[2];
    // gnssMsg->pose.pose.position.x = msg_in->gaussX - map_center_[0] ;
    // gnssMsg->pose.pose.position.y = msg_in->gaussY - map_center_[1] ;
    // gnssMsg->pose.pose.position.z = msg_in->height - map_center_[2]+80;
    gnssMsg->pose.pose.position.x = msg_in->gaussX - map_center_[0];
    gnssMsg->pose.pose.position.y = msg_in->gaussY - map_center_[1];
    gnssMsg->pose.pose.position.z = msg_in->height - map_center_[2];
    gnssMsg->pose.covariance[0] = msg_in->dev_gaussX;
    gnssMsg->pose.covariance[1] = msg_in->dev_gaussY;
    gnssMsg->pose.covariance[2] = msg_in->dev_height;
    gnssMsg->pose.covariance[3] = msg_in->dev_roll;
    gnssMsg->pose.covariance[4] = msg_in->dev_pitch;
    gnssMsg->pose.covariance[5] = msg_in->dev_azimuth;

    gnssMsg->pose.covariance[6] = msg_in->ins_status.ins_status;
    gnssMsg->pose.covariance[7] = msg_in->pos_type.pos_type;

    double roll  = msg_in->roll *M_PI/ 180;
    double pitch =-msg_in->pitch *M_PI/ 180;
    double yaw   = msg_in->azimuth *M_PI/ 180 ;

    
    gnssMsg->pose.pose.orientation =tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);


    mtx_buffer_.lock();

    if (timestamp < last_timestamp_gp_)
    {
        ROS_WARN("gp loop back, clear buffer");
        gnss_buffer_.clear();
    }

    while (gnss_buffer_.size()>200)
        gnss_buffer_.pop_front();
    

    last_timestamp_gp_ = timestamp;
    gnss_buffer_.push_back(gnssMsg);
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();
}
void Localization::localPose_cbk(const self_state::LocalPose::ConstPtr &msg_in) {
    //  LOG(INFO) << "get local pose.";

    self_state::LocalPose::Ptr msg(new self_state::LocalPose(*msg_in));
    double timestamp = msg->local_time;

    mtx_buffer_.lock();

    if (timestamp < last_timestamp_lp_)
    {
        ROS_WARN("lp loop back, clear buffer");
        lp_buffer_.clear();
    }

    while (lp_buffer_.size()>200)
        lp_buffer_.pop_front();

    last_timestamp_lp_ = timestamp;
    lp_buffer_.push_back(msg);
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();
}

void Localization::rvizPose_cbk(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    init_pose_msg_ = *msg;
    init_pose_received_ = true;
    LOG(INFO) << "Received initial pose from RViz: "
              << "x = " << msg->pose.pose.position.x
              << ", y = " << msg->pose.pose.position.y
              << ", z = " << msg->pose.pose.position.z;
}

void Localization::imuRawToRos(const sensor_msgs::Imu::ConstPtr &msg_raw, sensor_msgs::Imu::Ptr &msg)
{
    // msg->header = msg_raw->header;
    // msg->linear_acceleration.x =  msg_raw->linear_acceleration.y;
    // msg->linear_acceleration.y = -msg_raw->linear_acceleration.x;
    // msg->linear_acceleration.z =  msg_raw->linear_acceleration.z;
    // msg->angular_velocity.x =  msg_raw->angular_velocity.y * M_PI / 180.0;
    // msg->angular_velocity.y = -msg_raw->angular_velocity.x * M_PI / 180.0;
    // msg->angular_velocity.z =  msg_raw->angular_velocity.z * M_PI / 180.0;

    msg->header = msg_raw->header;
    msg->linear_acceleration.x =  msg_raw->linear_acceleration.y;
    msg->linear_acceleration.y =  -msg_raw->linear_acceleration.x;
    msg->linear_acceleration.z =  msg_raw->linear_acceleration.z;
    msg->angular_velocity.x =  msg_raw->angular_velocity.y * 10.0;
    msg->angular_velocity.y =  -msg_raw->angular_velocity.x * 10.0;
    msg->angular_velocity.z =  msg_raw->angular_velocity.z * 10.0;

    // msg->linear_acceleration.x = 0;
    // msg->linear_acceleration.y = 0;
    // msg->linear_acceleration.z = 9.8;
    // msg->angular_velocity.x = 0;
    // msg->angular_velocity.y = 0;
    // msg->angular_velocity.z = 0;

    // msg->header = msg_raw->header;
    // msg->linear_acceleration.x =  msg_raw->linear_acceleration.x;
    // msg->linear_acceleration.y =  msg_raw->linear_acceleration.y;
    // msg->linear_acceleration.z =  msg_raw->linear_acceleration.z;
    // msg->angular_velocity.x =  msg_raw->angular_velocity.x * M_PI / 180.0;
    // msg->angular_velocity.y =  msg_raw->angular_velocity.y * M_PI / 180.0;
    // msg->angular_velocity.z =  msg_raw->angular_velocity.z * M_PI / 180.0;

}

void Localization::PubLidarPose(state_group location_state)
{

    self_state::LidarGlobalPose lidar_pose;
    lidar_pose.local_time = lidar_end_time_*1000;
    lidar_pose.x = location_state.pos.x() + map_center_[0];
    lidar_pose.y = location_state.pos.y() + map_center_[1];
    lidar_pose.z = location_state.pos.z() + map_center_[2];

    double roll, pitch, yaw;
    Eigen::Quaterniond q_ = location_state.rot;
    tf::Matrix3x3(tf::Quaternion(q_.x(), q_.y(), q_.z(), q_.w())).getRPY(roll, pitch, yaw);
    
    lidar_pose.roll = roll*RAD_2_DEG;
    lidar_pose.pitch = pitch*RAD_2_DEG;
    lidar_pose.azimuth = yaw*RAD_2_DEG;

    lidar_pose.x_speed = p_map_->state_point.vel(0);
    lidar_pose.y_speed = p_map_->state_point.vel(1);
    lidar_pose.z_speed = p_map_->state_point.vel(2);

    pub_lidarPose_.publish(lidar_pose);



    self_state::GlobalPose fakeGPose_msg;
    fakeGPose_msg.local_time =lidar_end_time_*1000;
    fakeGPose_msg.gaussX = lidar_pose.x;
    fakeGPose_msg.gaussY = lidar_pose.y;
    fakeGPose_msg.height = lidar_pose.z;
    fakeGPose_msg.roll    = lidar_pose.roll;
    fakeGPose_msg.pitch   = lidar_pose.pitch;
    fakeGPose_msg.azimuth = lidar_pose.azimuth;
    fakeGPose_msg.vEast   = lidar_pose.x_speed;
    fakeGPose_msg.vNorth  = lidar_pose.y_speed;
    fakeGPose_msg.vUp     = lidar_pose.z_speed;
    XY_BLH::GaussProjInvCal(fakeGPose_msg.gaussX+BASE_X, fakeGPose_msg.gaussY+BASE_Y, &fakeGPose_msg.longitude, &fakeGPose_msg.latitude);
    pub_fakeGPose.publish(fakeGPose_msg);




    geometry_msgs::PoseStamped msg_body_pose;
    msg_body_pose.pose.position.x = location_state.pos(0);
    msg_body_pose.pose.position.y = location_state.pos(1);
    msg_body_pose.pose.position.z = location_state.pos(2);
    msg_body_pose.pose.orientation.x = q_.coeffs()[0];
    msg_body_pose.pose.orientation.y = q_.coeffs()[1];
    msg_body_pose.pose.orientation.z = q_.coeffs()[2];
    msg_body_pose.pose.orientation.w = q_.coeffs()[3];
    
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time_);
    msg_body_pose.header.frame_id = frame_body_;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 1 == 0)
    {
        path_vec_.poses.push_back(msg_body_pose);
        pub_path_.publish(path_vec_);
    }




    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(msg_body_pose.pose.position.x,
                                    msg_body_pose.pose.position.y,
                                    msg_body_pose.pose.position.z));
    q.setW(msg_body_pose.pose.orientation.w);
    q.setX(msg_body_pose.pose.orientation.x);
    q.setY(msg_body_pose.pose.orientation.y);
    q.setZ(msg_body_pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg_body_pose.header.stamp, frame_map_, frame_body_));

}



bool Localization::SyncPpackages(MeasureGroup &meas)
{
    double start = omp_get_wtime();
    

    static int scan_used_count = 0;
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        // LOG(WARNING)<<"lidar or imu is empty, lidar size : "<<lidar_buffer_.size()<<" imu size: "<<imu_buffer_.size();
        return false;
    }


    // if (!b_lidar_pushed_)
    // {
    //     meas.lidar = lidar_buffer_.front();
    //     lidar_end_time_ = meas.lidar_bag_time + meas.lidar->points.back().curvature / double(1000);
    //     lidar_bag_time_ = time_buffer_.front();
    //     meas.lidar_bag_time = lidar_bag_time_;  // 帧头时间
    //     meas.lidar_end_time = lidar_end_time_;

    //     b_lidar_pushed_ = true;
    // }

    if (!b_lidar_pushed_)
    {
        lidar_bag_time_ = time_buffer_.front();
        meas.lidar = lidar_buffer_.front();
        meas.lidar_bag_time = time_buffer_.front();

        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time_ = meas.lidar_bag_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {

            lidar_end_time_ = meas.lidar_bag_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time_ = meas.lidar_bag_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        std::cout<<"offset time :" << lidar_mean_scantime<<std::endl;
        meas.lidar_end_time = lidar_end_time_;

        b_lidar_pushed_ = true;

        // std::cout<<"time beg end :  "<< meas.lidar_bag_time<<" "<<meas.lidar_end_time<<" "<< meas.lidar->points.back().curvature / double(1000)<<std::endl;
    }

    if (last_timestamp_imu_ < lidar_end_time_)
    {
        LOG(WARNING)<<("no imu data behand lidar.");
        return false;
    }

//    |  |  | |  |      time imu
//     |          |       time cloud
//    在meas中，最前的imu时间早于 lidar_beg，最后的imu时间早于 lidar_end
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
    {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_)
            break;
        meas.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    b_lidar_pushed_ = false;
    double end =omp_get_wtime();

    // LOG(INFO) << "sync  cost time : " << (end - start)*1000 ;
    return true;
}


// V3D geometryqToEuler(geometry_msgs::Quaternion q)
// {
//     tf2::Quaternion tfq;
//     tf2::convert(q, tfq);

//     // 四元数转欧拉角
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);

//     return V3D(roll, pitch, yaw);

// }

void Localization::CalLocationError()
{
    if (gnss_buffer_.empty()) {
        LOG(WARNING) << "GNSS buffer is empty.";
        return ;
    }

    // Filter GNSS data within the acceptable time range
    bool b_get_gnss = false;
    nav_msgs::Odometry::Ptr cur_gnss;
    while (!gnss_buffer_.empty()) {
        double gnss_time = gnss_buffer_.front()->header.stamp.toSec();
        // LOG(INFO) << setprecision(15) << "lidar gnss time :" << gnss_time << " " << lidar_bag_time_;

        if (gnss_time < lidar_bag_time_) {
            gnss_buffer_.pop_front();
            continue;
        }
        if (gnss_time > lidar_end_time_) {
            break;
        }
        cur_gnss = gnss_buffer_.front();
        gnss_buffer_.pop_front();
        b_get_gnss = true;
    }

if( b_get_gnss ) 
{

        double dist_x = location_state.pos(0) - cur_gnss->pose.pose.position.x;
        double dist_y = location_state.pos(1) - cur_gnss->pose.pose.position.y;
        // double vehicle_x = location_state.pos(0) - 2.6*cos(yaw);
        // double vehicle_y = location_state.pos(1) - 2.6*sin(yaw);
        // double dist_x = vehicle_x - cur_gnss->pose.pose.position.x;
        // double dist_y = vehicle_y - cur_gnss->pose.pose.position.y;

        double dist = sqrt(dist_x * dist_x + dist_y * dist_y );
        LOG(INFO) << "LOC dist : " << dist;
        fout_error<<dist<<endl;


        Eigen::Quaterniond q_loc =  location_state.rot;
        Eigen::Vector3d angle_loc = SO3ToEuler(q_loc);
        // Eigen::Vector3d pose_loc {vehicle_x,vehicle_y,location_state.pos(2)};
        Eigen::Vector3d pose_loc = location_state.pos;
       // fout_loc<<pose_loc(0)<<" "<<pose_loc(1)<<" "<<pose_loc(2)+0.2<<" "<<angle_loc(0)<<" "<<angle_loc(1)<<" "<<angle_loc(2)<<endl;
        fout_loc<<pose_loc(0)<<" "<<pose_loc(1)<<" "<<pose_loc(2)<<" "<<angle_loc(0)<<" "<<angle_loc(1)<<" "<<angle_loc(2)<<endl;


        Eigen::Quaterniond q_gnss(
            cur_gnss->pose.pose.orientation.w,
            cur_gnss->pose.pose.orientation.x,
            cur_gnss->pose.pose.orientation.y,
            cur_gnss->pose.pose.orientation.z
        );
        Eigen::Vector3d angle_gnss = SO3ToEuler(q_gnss);
        Eigen::Vector3d pose_gnss(
            cur_gnss->pose.pose.position.x,
            cur_gnss->pose.pose.position.y,
            cur_gnss->pose.pose.position.z
        );
        fout_gnss<<pose_gnss(0)<<" "<<pose_gnss(1)<<" "<<pose_gnss(2)<<" "<<angle_gnss(0)<<" "<<angle_gnss(1)<<" "<<angle_gnss(2)<<endl;





    }
    else {
        LOG(INFO) << "No gnss data in the acceptable time range.";
    }

}

bool Localization::JudgeLocationState(state_group gnss_state, state_group location_state)
{

    // V3D localtion_rot = SO3ToEuler( p_map_->state_point.rot );
    // V3D localtion_pos = p_map_->state_point.pos ;

    // nav_msgs::Odometry::Ptr cur_gnss = gnss_buffer_.front();
    // V3D gnss_rot = geometryqToEuler(cur_gnss->pose.pose.orientation);
    // V3D gnss_pos = cur_gnss->pose.pose.position;

    // 设定阈值


    // 计算位置差异
    double pos_diff = (gnss_state.pos - location_state.pos).norm();

    // 计算旋转差异
    double rot_diff = gnss_state.rot.angularDistance(location_state.rot);

    // 判断是否超过阈值
    if (pos_diff > threshold_position_ || rot_diff > threshold_rotation_) {
        return true; // 需要重定位
    }

    return false; // 不需要重定位

}

bool Localization::GetGNSSMeasure(state_group &gnss_state)
{
     if (gnss_buffer_.empty()) {
        LOG(WARNING) << "GNSS buffer is empty.";
        return false;
    }

    // Filter GNSS data within the acceptable time range
    nav_msgs::Odometry::Ptr cur_gnss;
    while (!gnss_buffer_.empty()) {
        double gnss_time = gnss_buffer_.front()->header.stamp.toSec();
        LOG(INFO) << setprecision(15) << "lidar gnss time :" << gnss_time << " " << lidar_bag_time_;

        if (gnss_time < lidar_bag_time_) {
            gnss_buffer_.pop_front();
            continue;
        }
        if (gnss_time > lidar_end_time_) {
            break;
        }
        cur_gnss = gnss_buffer_.front();
        gnss_buffer_.pop_front();
    }

    if (cur_gnss==nullptr) {
        LOG(WARNING) << "No usable GNSS data within the time range.";
        return false;
    }

    // Check covariance to ensure data quality, if needed
    if (cur_gnss->pose.covariance[6] != 3) {
        LOG(WARNING) << "GNSS has not converged. ins_status : " << cur_gnss->pose.covariance[6]
                                         << " pose_type : "  << cur_gnss->pose.covariance[7];
        return false;
    }

    if (cur_gnss->pose.covariance[0] >5 || cur_gnss->pose.covariance[1] >5) {
        LOG(WARNING) << "GNSS covariance is too high .  dev_x : " << cur_gnss->pose.covariance[0]
                                                << " dev_y : " << cur_gnss->pose.covariance[1];
        return false;
    }

    
    // Set GNSS state
    gnss_state.pos = Eigen::Vector3d(cur_gnss->pose.pose.position.x, cur_gnss->pose.pose.position.y, cur_gnss->pose.pose.position.z);
    gnss_state.rot = Eigen::Quaterniond(cur_gnss->pose.pose.orientation.w, cur_gnss->pose.pose.orientation.x,
                         cur_gnss->pose.pose.orientation.y, cur_gnss->pose.pose.orientation.z);
    // tf2::convert(cur_gnss->pose.pose.orientation, gnss_state.rot);
    LOG(INFO) << "GNSS converted sucessd . " ;

    return true;

}


state_group GetInitState(vector<double> xyzrpy, vector<double> map_center)
{
    state_group state;
    state.pos << xyzrpy[0]-map_center[0], xyzrpy[1]-map_center[1], xyzrpy[2]-map_center[2];
    state.rot =  EulerToSO3(xyzrpy[3]*DEG_2_RAD, xyzrpy[4]*DEG_2_RAD, xyzrpy[5]*DEG_2_RAD);
    return state;
    
}

state_group GetInitStateFromFirstGNSS(nav_msgs::Odometry::Ptr cur_gnss)
{
    state_group init_state;
    init_state.pos =  Eigen::Vector3d(cur_gnss->pose.pose.position.x, cur_gnss->pose.pose.position.y, cur_gnss->pose.pose.position.z);
    init_state.rot =  Eigen::Quaterniond(cur_gnss->pose.pose.orientation.w, cur_gnss->pose.pose.orientation.x,
                         cur_gnss->pose.pose.orientation.y, cur_gnss->pose.pose.orientation.z);
    return init_state;
}

void Localization::AddMapToViewer(PointCloudXYZI::Ptr map, Eigen::Vector3d pos, pcl::visualization::PCLVisualizer::Ptr viewer) {
    if( !b_display_init_ )
        return;

        // viewer.reset(new pcl::visualization::PCLVisualizer ("Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->setCameraPosition(pos[0]-150, pos[1]-150, pos[2]+500, pos[0], pos[1], pos[2], 0, 0, 1);


    // 添加map点云到视图，只添加一次
    pcl::visualization::PointCloudColorHandlerCustom<PointType> map_color_handler(map, 128, 128, 128);
    viewer->addPointCloud<PointType>(map, map_color_handler, "map cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map cloud");
    viewer->spin();
}

Eigen::Vector3d last_pos;
void Localization::AddFrameToViewer(PointCloudXYZI::Ptr frame, Eigen::Vector3d pos , pcl::visualization::PCLVisualizer::Ptr viewer) {

    if( !b_display_init_ )
        return;

    // 检查并移除旧的frame点云
    if (viewer->contains("frame cloud")) {
        viewer->removePointCloud("frame cloud");
    }

    static bool b_first = true;
    static int idx=0;

    if(b_first ) {
        b_first = false;
        last_pos = pos;
    }
    else{
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(
            pcl::PointXYZ(last_pos[0], last_pos[1], last_pos[2]),  // 上一位置的点
            pcl::PointXYZ(pos[0], pos[1], pos[2]),  // 当前位置的点
            "line" + std::to_string(idx)  // 线段的ID，确保每次都是唯一的
        );
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.9, 0.9, "line" + std::to_string(idx));  // 设置颜色为靛青色
        idx++;

    }
        
    // viewer->setCameraPosition(pos[0]-150, pos[1]-150, pos[2]+500, pos[0], pos[1], pos[2], 0, 0, 100);
    // viewer->setCameraPosition(pos[0]-150, pos[1]-150, pos[2]+500, pos[0], pos[1], pos[2], 0, 0, 1);


    // 添加或更新frame点云到视图，根据高程（Z值）着色
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> frame_color_handler(frame, "z");
    viewer->addPointCloud<PointType>(frame, frame_color_handler, "frame cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame cloud");
    // 更新视图
    viewer->spinOnce(10);

    last_pos = pos;

    // viewer->spin();
}

void Localization::AddFrameToViewer(PointCloudXYZI::Ptr frame,  pcl::visualization::PCLVisualizer::Ptr viewer) {
    if( !b_display_init_ )
        return;
        
        // 检查并移除旧的frame点云
    if (viewer->contains("frame cloud")) {
        viewer->removePointCloud("frame cloud");
    }




    // 添加或更新frame点云到视图，根据高程（Z值）着色
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> frame_color_handler(frame, "z");
    viewer->addPointCloud<PointType>(frame, frame_color_handler, "frame cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame cloud");

    // 更新视图
    viewer->spinOnce(10);
    // viewer->spin();
}

void downsampleAndSavePointCloud(PointCloudXYZI::Ptr map, const std::string& filename) {
    // 创建降采样后的点云对象
    PointCloudXYZI::Ptr cloud_filtered(new PointCloudXYZI);

    // 设置VoxelGrid滤波器
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(map);
    // sor.setLeafSize(0.5f, 0.5f, 0.5f);  // 设置体素网格的大小为0.5 x 0.5 x 0.5
    sor.setLeafSize(1.0f, 1.0f, 1.0f);  // 设置体素网格的大小为0.5 x 0.5 x 0.5
    sor.filter(*cloud_filtered);

    // 保存降采样后的点云到本地PCD文件
    pcl::io::savePCDFileBinary(filename, *cloud_filtered);
    std::cout << "Saved downsampled point cloud to " << filename << ". File has " << cloud_filtered->size() << " points." << std::endl;
}



void Localization::ShowInitResult(pcl::visualization::PCLVisualizer::Ptr vis, 
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr frame, 
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr map,
                                  string window_name )
{
    // vis.reset(new pcl::visualization::PCLVisualizer (window_name));
            
    vis->setBackgroundColor(0, 0, 0);
    vis->setCameraPosition(-50, -50, 500, 0, 0, 0, 0, 0, 1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> map_color_handler(map, 128, 128, 128);
    vis->addPointCloud<pcl::PointXYZ>(map, map_color_handler, "map cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map cloud");

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> frame_color_handler(frame, "z");
    vis->addPointCloud<pcl::PointXYZ>(frame, frame_color_handler, "frame cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "frame cloud");

    vis->addCoordinateSystem(10.0);
    vis->spin();

    vis->removeAllPointClouds();
    vis->removeAllShapes();
    vis->close();

}

void Localization::ShowMatchResultDual(pcl::visualization::PCLVisualizer::Ptr vis, 
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map,
                                  string window_name )
{
    // vis.reset(new pcl::visualization::PCLVisualizer (window_name));

    int v1(0);
    int v2(1);

    vis->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    vis->createViewPort(0.5, 0,   1.0, 1.0, v2);

    Eigen::Matrix4f Tr;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> current_h (cloud_first, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> current_h2 (cloud_second, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> last_h (cloud_map, 0, 255, 0);


    vis->addPointCloud (cloud_first, current_h, "vp2_source1", v1);
    vis->addPointCloud (cloud_map,   last_h,    "vp2_target1", v1);
    vis->addPointCloud (cloud_second, current_h2, "vp2_source2", v2);
    vis->addPointCloud (cloud_map,    last_h,     "vp2_target2", v2);

    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_source1");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_source2");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_target1");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp2_target2");

    vis->spin();

    vis->removeAllPointClouds();
    vis->removeAllShapes();
    vis->close();

}

void Localization::GetHandInitPose(state_group &init_state,PointCloudXYZI::Ptr map, PointCloudXYZI::Ptr frame)
{
    Eigen::Affine3d Aff_init = Eigen::Affine3d::Identity();
    Aff_init.rotate(init_state.rot.matrix());
    Aff_init.translation() = init_state.pos;



    int map_rows = 500, map_cols = 500;

//    float MIN_Z = -5, MAX_Z = 5;
    float MIN_Z = -3,   MAX_Z = 3;
    float MIN_X = -50,  MAX_X = 50;

    cv::Mat show_img1 = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    cv::Mat show_img2 = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);

    double point_x, point_y, point_z;
    int img_x, img_y;
    int r, g, b;
    int cell_size = 3;

    pcl::PointXYZ p_in, p_out;
    Eigen::Vector3d initial_pose = init_state.pos;
    Eigen::Vector3d initial_euler = init_state.rot.matrix().eulerAngles(0,1,2);

    //  = init_state.pos;
    for(int i=0; i<map->size(); i++) {
        point_x =  map->points[i].x - initial_pose[0];   // loam to our coordinate
        point_y =  map->points[i].y - initial_pose[1];
        point_z =  map->points[i].z - initial_pose[2];

        img_x = (point_x * cell_size) + 250;
        img_y = 250 - (point_y * cell_size);
        if( img_x <0 || img_y<0 || img_x>=500 || img_y >=500 )
            continue;
        double ratio = double(point_z - MIN_Z) / double(MAX_Z - MIN_Z);
        if(ratio<0) ratio = 0;
        if(ratio>1) ratio = 1;
        int color = 50+ ratio * 150 ;

        show_img1.at<cv::Vec3b>(img_y,img_x) = cv::Vec3b(color,color,color);

        img_x = 250 - (point_y * cell_size);
        img_y = 250 - (point_z * cell_size);
        if( img_x <0 || img_y<0 || img_x>=500 || img_y >=500 )
            continue;
        ratio = double(point_x - MIN_X) / double(MAX_X - MIN_X);
        if(ratio<0) ratio = 0;
        if(ratio>1) ratio = 1;
        color = ratio * 200 ;
        show_img2.at<cv::Vec3b>(img_y,img_x) = cv::Vec3b(color,color,color);
    }

    Eigen::Matrix4f T_manual;
    Eigen::Matrix3f R_manual;
    Eigen::Vector3f t_manual;

    double ori_x = initial_pose[0], ori_y = initial_pose[1], ori_z = initial_pose[2];
    bool b_correct1 = true,   b_correct2 = false;

    while(b_correct1){
        cv::Mat show_img_cp = show_img1.clone();

        int c = cv::waitKey(-1);
        T_manual.setIdentity();

        switch (c){
        case 81:        // Q
//            R_manual = Eigen::AngleAxisf(0.3*M_PI/180.0, Eigen::Vector3f::UnitZ());
//            T_manual.topLeftCorner(3,3) = R_manual;
            initial_euler[2] += 0.3*M_PI/180.0;
            std::cout<<"rotation left\n";
            break;
        case 69:        // E
//            R_manual = Eigen::AngleAxisf(-0.3*M_PI/180.0, Eigen::Vector3f::UnitZ());
//            T_manual.topLeftCorner(3,3) = R_manual;
            initial_euler[2] -= 0.3*M_PI/180.0;
            std::cout<<"rotation right\n";

            break;
        case 87:        // W
//            t_manual << 0, 0.1, 0;
//            T_manual.topRightCorner(3,1) = t_manual;
            initial_pose[1] += 0.05;
            std::cout<<"translate up\n";

            break;
        case 65:        // A
//            t_manual << -0.1, 0, 0;
//            T_manual.topRightCorner(3,1) = t_manual;
            initial_pose[0] -= 0.05;
            std::cout<<"translate left\n";

            break;
        case 83:        // S
//            t_manual << 0, -0.1, 0;
//            T_manual.topRightCorner(3,1) = t_manual;
            initial_pose[1] -= 0.05;
            std::cout<<"translate down\n";

            break;
        case 68:        // D
//            t_manual << 0.1, 0, 0;
//            T_manual.topRightCorner(3,1) = t_manual;
            initial_pose[0] += 0.05;
            std::cout<<"translate right\n";

            break;
        case 13:
            b_correct1 = false;
            b_correct2 = true;
            break;

        default:
            break;
        }



        Eigen::Affine3d Aff_transform = Eigen::Affine3d::Identity();
        Aff_transform.rotate(Eigen::AngleAxisd(initial_euler[0], Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(initial_euler[1], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(initial_euler[2], Eigen::Vector3d::UnitZ()));
        Aff_transform.translation() = initial_pose;

        for( int i=0; i<frame->size(); i++ ) {
            p_in.x = frame->points[i].x;
            p_in.y = frame->points[i].y;
            p_in.z = frame->points[i].z;
            p_out =  pcl::transformPoint(p_in, Aff_transform);

            point_x =  p_out.x - ori_x;
            point_y =  p_out.y - ori_y;
            point_z =  p_out.z - ori_z;

            // std::cout<<point_x<<" "<<point_y<<" "<<point_z<<endl;

            img_x = (point_x * cell_size) + 250;
            img_y = 250 - (point_y * cell_size);
            if( img_x <0 || img_y<0 || img_x>=500 || img_y >=500 )
                continue;
            double ratio = double(point_z - MIN_Z) / double(MAX_Z - MIN_Z);
            if(ratio<0) ratio = 0;
            if(ratio>1) ratio = 1;
            int color_idx = 300 + ratio * 320;
            r = jet_color_map[color_idx][2];
            g = jet_color_map[color_idx][1];
            b = jet_color_map[color_idx][0];

            show_img_cp.at<cv::Vec3b>(img_y,img_x) = cv::Vec3b(r,g,b);
        }
        cv::namedWindow("Localization",cv::WINDOW_NORMAL);
        cv::imshow("Localization",show_img_cp);
    }
    cv::namedWindow("Localization",cv::WINDOW_NORMAL);
    cv::imshow("Localization",show_img2);


    while(b_correct2){
        cv::Mat show_img_cp = show_img2.clone();

        int c = cv::waitKey(-1);


        switch (c){
        case 87:        // W
//            t_manual << 0, 0.1, 0;
//            T_manual.topRightCorner(3,1) = t_manual;
            initial_pose[2] += 0.05;
            std::cout<<"translate up\n";

            break;
        case 83:        // S
//            t_manual << 0, -0.1, 0;
//            T_manual.topRightCorner(3,1) = t_manual;
            initial_pose[2] -= 0.05;
            std::cout<<"translate down\n";

            break;
        case 13:
            b_correct2 = false;
            break;

        default:
            break;
        }

        Eigen::Affine3d Aff_transform = Eigen::Affine3d::Identity();
        Aff_transform.rotate(Eigen::AngleAxisd(initial_euler[0], Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(initial_euler[1], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(initial_euler[2], Eigen::Vector3d::UnitZ()));
        Aff_transform.translation() = initial_pose;

        for( int i=0; i<frame->size(); i++ ) {
            p_in.x = frame->points[i].x;
            p_in.y = frame->points[i].y;
            p_in.z = frame->points[i].z;
            p_out =  pcl::transformPoint(p_in, Aff_transform);

            point_x =  p_out.x - ori_x;
            point_y =  p_out.y - ori_y;
            point_z =  p_out.z - ori_z;

            img_x = 250 - (point_y * cell_size);
            img_y = 250 - (point_z * cell_size);
            if( img_x <0 || img_y<0 || img_x>=500 || img_y >=500 )
                continue;
            double ratio = double(point_x - MIN_X) / double(MAX_X - MIN_X);
            if(ratio<0) ratio = 0;
            if(ratio>1) ratio = 1;
            int color_idx = 300 + ratio * 320;
            r = jet_color_map[color_idx][2];
            g = jet_color_map[color_idx][1];
            b = jet_color_map[color_idx][0];

            show_img_cp.at<cv::Vec3b>(img_y,img_x) = cv::Vec3b(r,g,b);
        }

        cv::namedWindow("Localization",cv::WINDOW_NORMAL);
        cv::imshow("Localization",show_img_cp);
        //        cv::waitKey(1);
    }

    geometry_msgs::Quaternion q;
    q=tf::createQuaternionMsgFromRollPitchYaw(initial_euler[0], initial_euler[1], initial_euler[2]);
    init_state.rot = SO3(q.w, q.x, q.y, q.z);
    init_state.pos = initial_pose;

}


void Localization::GetAutoInitPose(state_group &init_state,PointCloudXYZI::Ptr map, PointCloudXYZI::Ptr frame)
{

    bool b_init_twice = true;
    bool b_display_init = true;

    Eigen::Affine3d Aff_imu_lidar = Eigen::Affine3d::Identity();
    Aff_imu_lidar.rotate(p_imu_->Lidar_R_wrt_IMU);
    Aff_imu_lidar.translation() = p_imu_->Lidar_T_wrt_IMU;

    Eigen::Affine3d Aff_world_imu = Eigen::Affine3d::Identity();
    Aff_world_imu.rotate(init_state.rot.matrix());
    Aff_world_imu.translation() = init_state.pos;

    Eigen::Affine3d Aff_imu_world = Aff_world_imu.inverse();
    Eigen::Affine3f Aff_imu_world_f = Aff_imu_world.cast<float>();
    Eigen::Affine3f Aff_imu_lidar_f = Aff_imu_lidar.cast<float>();

    // Eigen::Matrix4d Tr_imu_world = Aff_imu_world.matrix();
    // Eigen::Matrix4d Tr_imu_lidar = Aff_imu_lidar.matrix();
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_imu(new pcl::PointCloud<pcl::PointXYZ>);



    double point_x, point_y, point_z;
    double dist;
    float range_thresh_pow = 80*80;



    pcl::PointXYZ p_in, p_out;

    std::cout<<"frame size 1 : " << frame->size()<<std::endl;

    for( auto &pt : *frame) {
        p_in.x = pt.x;
        p_in.y = pt.y;
        p_in.z = pt.z;
        p_out = pcl::transformPoint(p_in, Aff_imu_lidar_f);  // lidar -> imu

        // if( pow(p_out.x, 2) + pow(p_out.y, 2) < range_thresh_pow ) 
            frame_imu->push_back(p_out);
    }
    for( auto &pt : *map) {
        p_in.x = pt.x;
        p_in.y = pt.y;
        p_in.z = pt.z;
        p_out = pcl::transformPoint(p_in, Aff_imu_world_f);   // world -> imu

        if( pow(p_out.x, 2) + pow(p_out.y, 2) < range_thresh_pow ) 
            map_imu->push_back(p_out);
    }


    std::cout<<"frame size 2 : " << frame_imu->size()<<std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilterSurf.setInputCloud(frame_imu);
    downSizeFilterSurf.filter(*frame_imu);
//    downSizeFilterSurf.setInputCloud(cloud_map);
//    downSizeFilterSurf.filter(*cloud_map);

    std::cout<<"frame size 3 : " << frame_imu->size()<<std::endl;

    if( b_display_init_ ) 
        ShowInitResult(vis, frame_imu, map_imu, "gnss_result");


    std::cout<<"init map size : "<<map_imu->points.size()<<std::endl;
    std::cout<<"init frame size : "<<frame_imu->points.size()<<std::endl;


    LOG(INFO) << "start matching .";
    Eigen::Matrix4f Tr_delta = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tr_delta_tmp = Eigen::Matrix4f::Identity();

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> *ndt;
    ndt = new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

    ndt->setTransformationEpsilon (0.00001);    //为终止条件设置最小转换差异
    ndt->setStepSize (0.1);                     //为More-Thuente线搜索设置最大步长
    ndt->setResolution (1.0);                  //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt->setMaximumIterations (1000);           //设置匹配迭代的最大次数
    ndt->setInputSource(frame_imu);
    ndt->setInputTarget(map_imu);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ndt (new pcl::PointCloud<pcl::PointXYZ>);
    ndt->align (*output_cloud_ndt);
    Tr_delta_tmp = ndt->getFinalTransformation();
    Tr_delta = Tr_delta_tmp;
    double ndt_score = ndt->getFitnessScore();

    LOG(INFO) << "end first matching .";

    if( b_init_twice ) {
        // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> *ndt2;
        // ndt2 = new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
        // ndt2->setTransformationEpsilon (0.00001);    //为终止条件设置最小转换差异
        // ndt2->setStepSize (0.1);                     //为More-Thuente线搜索设置最大步长
        // ndt2->setResolution (1);                     //设置NDT网格结构的分辨率（VoxelGridCovariance）
        // ndt2->setMaximumIterations (100);            //设置匹配迭代的最大次数
        // ndt2->setInputSource(cloud_cur);
        // ndt2->setInputTarget(cloud_map);
        // output_cloud_ndt.reset(new pcl::PointCloud<pcl::PointXYZ>());
        // ndt2->align (*output_cloud_ndt, Tr_delta );
        // Tr_delta = ndt2->getFinalTransformation();

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> *icp;
        icp = new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;

        icp->setTransformationEpsilon(0.00001);    // 设置收敛条件下的最小变换差
        icp->setMaxCorrespondenceDistance(0.1);    // 设置最大对应点对距离
        icp->setMaximumIterations(100);            // 设置最大迭代次数
        icp->setInputSource(frame_imu);            // 设置输入点云（源）
        icp->setInputTarget(map_imu);            // 设置目标点云

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_icp(new pcl::PointCloud<pcl::PointXYZ>());
        icp->align(*output_cloud_icp, Tr_delta);
        Tr_delta = icp->getFinalTransformation();
        double icp_score = icp->getFitnessScore();

        LOG(INFO) << "socre ndt icp : " << ndt_score<<" "<<icp_score;

    }
    LOG(INFO) << "end matching .";

    // std::cout << "Tr delta : \n " << Tr_delta;

    Eigen::Affine3d Aff_delta =  Eigen::Affine3d(Tr_delta.cast<double>());

    std::cout << "Tr before : \n " << Aff_world_imu.matrix();
    Aff_world_imu = Aff_world_imu * Aff_delta;
    std::cout << "Tr after : \n " << Aff_world_imu.matrix() << endl;

    init_state.rot = Aff_world_imu.rotation();
    init_state.pos = Aff_world_imu.translation();


    if( b_display_init_ ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu_corrected(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*frame_imu, *frame_imu_corrected, Tr_delta_tmp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu_corrected2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*frame_imu, *frame_imu_corrected2, Tr_delta);

        // ShowInitResult(vis, frame_imu_corrected, map_imu,  "match_first_result");
        // ShowInitResult(vis, frame_imu_corrected2, map_imu, "match_second_result");
        ShowMatchResultDual( vis, frame_imu_corrected, frame_imu_corrected2, map_imu, "matching_dual" );
    }



}


bool Localization::InitPose()
{
    if( gnss_buffer_.empty() ) 
        return false;


    gnss_state = GetInitStateFromFirstGNSS(gnss_buffer_.back());

    LOG(INFO) << "Before match gnss_state : " << gnss_state.pos;
    GetAutoInitPose(gnss_state, p_map_->cloud_map, Measures_.lidar);
    LOG(INFO) << "After match gnss_state : " << gnss_state.pos;

    return true;
}


/*
// void Localization::ShowInSattle()
// {
//     static bool b_first = true;

//     if( b_first ) {
//         b_first = false;
//         return ;
//     }

//     bool show_in_ros = false;
//     bool show_cloud = false;

//     cv::Point2d gauss, pixel;

//     if( show_cloud ) {
//         float x, y, z;
//         float intensity;
//         float max_z = 5, min_z = -2;
//         for( int i=0; i<p_map_->feats_down_world->size(); i++ ) {
//             gauss.x = p_map_->feats_down_world->points[i].x + map_center_[0] + BASE_X;
//             gauss.y = p_map_->feats_down_world->points[i].y + map_center_[1] + BASE_Y;
//             z = p_map_->feats_down_body->points[i].z;
//             z = std::min(z, max_z);
//             z = std::max(z, min_z);

//             pixel = sattle_utiler.Gauss2Pixel(gauss);

//             intensity = p_map_->feats_down_world->points[i].intensity;

//             int col = static_cast<int>(pixel.x);  
//             int row = static_cast<int>(pixel.y);  


//             if (row >= 0 && row < intensity_mat.rows && col >= 0 && col < intensity_mat.cols) {
//                 float& current_intensity = intensity_mat.at<float>(row, col);

//                 if (intensity > current_intensity) {
//                     current_intensity = intensity;

//                     // uchar mapped_intensity = static_cast<uchar>(intensity);
//                     uchar mapped_intensity = static_cast<uchar>( (z-min_z) / (max_z - min_z) * 255 );

//                     // 使用伪彩色映射，将强度值转为 RGB
//                     cv::Mat intensity_color(1, 1, CV_8UC1, cv::Scalar(mapped_intensity));
//                     cv::Mat rgb_color;
//                     cv::applyColorMap(intensity_color, rgb_color, cv::COLORMAP_JET);

                    
//                     sattle_mat.at<cv::Vec3b>(row, col) = rgb_color.at<cv::Vec3b>(0, 0);
//                 }
//             }

//         }
//     }
//     else {
//         gauss.x = location_state.pos(0) + map_center_[0] + BASE_X;
//         gauss.y = location_state.pos(1) + map_center_[1] + BASE_Y;
//         pixel = sattle_utiler.Gauss2Pixel(gauss);

//         int cur_col = static_cast<int>(pixel.x);  
//         int cur_row = static_cast<int>(pixel.y);  

//         gauss.x = pre_location_state.pos(0) + map_center_[0] + BASE_X;
//         gauss.y = pre_location_state.pos(1) + map_center_[1] + BASE_Y;
//         pixel = sattle_utiler.Gauss2Pixel(gauss);

//         int pre_col = static_cast<int>(pixel.x);
//         int pre_row = static_cast<int>(pixel.y);

//         if (cur_row >= 0 && cur_row < sattle_mat.rows && cur_col >= 0 && cur_col < sattle_mat.cols) {
//             // sattle_mat.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);  // 将像素设置为绿色
//             // cv::circle(sattle_mat, cv::Point(col, row), 3, cv::Scalar(0, 255, 0), -1);
//             cv::line(sattle_mat, cv::Point(pre_col, pre_row), cv::Point(cur_col, cur_row), cv::Scalar(0, 255, 0), 2);
//         }

//     }

//     cv::Mat sattle_mat_cp = sattle_mat.clone();


//     if (sattle_mat_cp.empty()) {
//         ROS_WARN("Input image is empty. Not publishing.");
//         return;  // 如果为空，直接返回
//     }


//     if( show_in_ros ) {

//         sensor_msgs::CompressedImage msg;
        
//         msg.format = "jpeg"; // 或者 "png"

//         std::vector<uchar> buf;
//         cv::imencode(".jpg", sattle_mat_cp, buf);  // 如果你想要 PNG 格式，可以改为 ".png"

//         msg.data.assign(buf.begin(), buf.end());
//         msg.header.stamp = ros::Time::now();
//         msg.header.frame_id = "map";
//         pub_result_mat.publish(msg);
//     }
//     else {
//         cv::resize(sattle_mat, sattle_mat_cp, cv::Size(800, 800));
//         cv::namedWindow("sattle", cv::WINDOW_NORMAL);
//         cv::imshow("sattle", sattle_mat_cp);
//         cv::waitKey(1);
//     }

// }
*/
void Localization::PublishCloud(PointCloudXYZI::Ptr map, PointCloudXYZI::Ptr frame)
{
    // static int idx = 0;
    // idx++;
    // if( idx%100 == 0 ) {
        // sensor_msgs::PointCloud2 msg_map;
        // pcl::toROSMsg(*map, msg_map);
        // msg_map.header.stamp =  ros::Time(lidar_end_time_);
        // msg_map.header.frame_id = "map";
        // pub_map_.publish(msg_map);
    // }


    sensor_msgs::PointCloud2 msg_frame;
    pcl::toROSMsg(*frame, msg_frame);
    msg_frame.header.stamp =  ros::Time(lidar_end_time_);
    msg_frame.header.frame_id = frame_map_;
    pub_frame_.publish(msg_frame);

}

void Localization::PubOdometry(state_group location_state)
{
    const Eigen::Vector3d pos = location_state.pos;
    const Eigen::Quaterniond rot = location_state.rot;
    const ros::Time stamp = ros::Time().fromSec(lidar_end_time_);

    // Keep the ROS output contract identical to FAST_LIO_robot: /lio/odom
    // uses "world", while /state_estimation and the corresponding TF use
    // "map" -> "sensor" with exactly the same timestamp and pose.
    nav_msgs::Odometry lio_odometry;
    lio_odometry.header.stamp = stamp;
    lio_odometry.header.frame_id = frame_world_;
    lio_odometry.child_frame_id.clear();
    lio_odometry.pose.pose.position.x = pos.x();
    lio_odometry.pose.pose.position.y = pos.y();
    lio_odometry.pose.pose.position.z = pos.z();
    lio_odometry.pose.pose.orientation.x = rot.x();
    lio_odometry.pose.pose.orientation.y = rot.y();
    lio_odometry.pose.pose.orientation.z = rot.z();
    lio_odometry.pose.pose.orientation.w = rot.w();
    lio_odometry.twist.twist.linear.x = p_map_->state_point.vel(0);
    lio_odometry.twist.twist.linear.y = p_map_->state_point.vel(1);
    lio_odometry.twist.twist.linear.z = p_map_->state_point.vel(2);

    const auto covariance = p_map_->kf.get_P();
    for (int i = 0; i < 6; ++i)
    {
        const int k = i < 3 ? i + 3 : i - 3;
        lio_odometry.pose.covariance[i * 6 + 0] = covariance(k, 3);
        lio_odometry.pose.covariance[i * 6 + 1] = covariance(k, 4);
        lio_odometry.pose.covariance[i * 6 + 2] = covariance(k, 5);
        lio_odometry.pose.covariance[i * 6 + 3] = covariance(k, 0);
        lio_odometry.pose.covariance[i * 6 + 4] = covariance(k, 1);
        lio_odometry.pose.covariance[i * 6 + 5] = covariance(k, 2);
    }
    pub_lio_odom_.publish(lio_odometry);

    nav_msgs::Odometry state_estimation = lio_odometry;
    state_estimation.header.frame_id = frame_map_;
    state_estimation.child_frame_id = frame_sensor_;
    pub_odometry.publish(state_estimation);

    tf::StampedTransform odomTrans;
    odomTrans.stamp_ = stamp;
    odomTrans.frame_id_ = frame_map_;
    odomTrans.child_frame_id_ = frame_sensor_;
    odomTrans.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
    odomTrans.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));
    tfBroadcaster.sendTransform(odomTrans);

    geometry_msgs::PoseStamped map_pose;
    map_pose.header.stamp = stamp;
    map_pose.header.frame_id = frame_map_;
    map_pose.pose = state_estimation.pose.pose;

    // Preserve the original /world_state/path output used by robot_loc.rviz.
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path_vec_.poses.push_back(map_pose);
        pub_path_.publish(path_vec_);
    }

    // Also expose the FAST_LIO-compatible path interface.
    static Eigen::Vector3d last_lio_path_position(0.0, 0.0, -100.0);
    if ((pos - last_lio_path_position).norm() > 0.1)
    {
        lio_path_vec_.poses.push_back(map_pose);
        pub_lio_path_.publish(lio_path_vec_);
        last_lio_path_position = pos;
    }
}

void Localization::Run_rviz()
{

    LOG(INFO) << "start run .";
    b_flg_exit_ = false;
    signal(SIGINT, Localization::SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();


    double first_lidar_time;

    sensor_msgs::PointCloud2 msg_map;
    
    pcl::toROSMsg(*(p_map_->cloud_map), msg_map);

    msg_map.header.stamp =  ros::Time(lidar_end_time_);
    msg_map.header.frame_id = frame_map_;
    pub_map_.publish(msg_map);
    LOG(INFO) << "Publish map ! map size : " << p_map_->cloud_map->size();



    bool b_first_run_ = true, b_flg_first_scan_ = true;
    bool b_need_init = true;
    LOG(INFO) << "START LOOP ! ";

    while (status)
    {
        ros::spinOnce();


        if (SyncPpackages(Measures_)){

            LOG(INFO) << "measure lidar size : " << Measures_.lidar->size();
            if( Measures_.lidar->size()==0 )
                continue;

            double start, end;
            start = omp_get_wtime();

            if (b_flg_first_scan_)  {


                if (init_pose_received_) {
                    gnss_state.pos << init_pose_msg_.pose.pose.position.x,
                                    init_pose_msg_.pose.pose.position.y,
                                    init_pose_msg_.pose.pose.position.z + z_offset_;

                    const auto& q_msg = init_pose_msg_.pose.pose.orientation;
                    Eigen::Quaterniond q_eigen(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
                    q_eigen.normalize();  

                    gnss_state.rot = q_eigen;



                    LOG(INFO) << "Before match gnss_state : " << gnss_state.pos;
                    GetAutoInitPose(gnss_state, p_map_->cloud_map, Measures_.lidar);
                    LOG(INFO) << "After match gnss_state : " << gnss_state.pos;
                

                    p_map_->SetPose(gnss_state);
                    init_pose_received_ = false;    

                    LOG(INFO) << "Finish init by rviz ";
                    
                }
                else{
                    LOG(WARNING) << "Waiting for initial pose from RViz...";
                    continue;
                }

                // double time_s = omp_get_wtime();
                // PointType point;
                // point.x = gnss_state.pos(0), point.y= gnss_state.pos(1), point.z = gnss_state.pos(2);
                // PointVector Storage;
                // p_map_->ikdtree.Radius_Search(point, 100, Storage);
                // LOG(INFO) << "Radius search cost time : " << (omp_get_wtime() - time_s)*1000 << " ms";


                // AddMapToViewer(p_map_->cloud_map, gnss_state.pos, vis);

                first_lidar_time = Measures_.lidar_bag_time;
                p_imu_->first_lidar_time = first_lidar_time;
                b_flg_first_scan_ = false;
                continue;
            }


            // std::cout<<"meas imu size : " << Measures_.imu.size()  << std::endl;
            p_imu_->Process(Measures_, p_map_->kf, cloud_frame_);


            cloud_frame_less_->clear();
            for( size_t i=0; i< cloud_frame_->size(); i++) 
                if (i %  point_filter_num_ == 0 ) 
                    cloud_frame_less_->emplace_back(cloud_frame_->points[i]);
// {
//             static int indx = 0;
//             if( !cloud_frame_less_->empty() ){
//                 std::cout<<"point num : "<< cloud_frame_->points.size() <<" less num : "<<cloud_frame_less_->points.size() <<std::endl;
//                 pcl::io::savePCDFileBinary("/home/tracer/workspace/code/c++/code2024/UGV/UGV2023_Green/program/module_modeling/bin/pcd_tmp1/" + to_string((lidar_end_time_*1000)) + ".pcd", *cloud_frame_);

//             }
// }


            LOG(INFO) << "points size ori : " <<cloud_frame_->size();

            if( p_map_->Process(cloud_frame_less_) ) {
                p_map_->GetPose(location_state);

                PubOdometry(location_state);
                PublishCloud(p_map_->ikdtree_cloud, p_map_->feats_undistort_world);

                pre_location_state = location_state;
            }

            end = omp_get_wtime();
            LOG(INFO) << "one loop cost time : " << (end - start)*1000 << "\n\n";
        }



        if (b_flg_exit_)     break;
        status = ros::ok();
        rate.sleep();
    }
}


void Localization::Run_gnss()
{

    LOG(INFO) << "start run .";
    b_flg_exit_ = false;
    signal(SIGINT, Localization::SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();


    double first_lidar_time;

    if(0) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        string color_map_path = "/home/tracer/workspace/data/Map/Hexi/hexi_rgb_ds_0.5.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(color_map_path, *color_cloud) == -1) {
            LOG(ERROR) <<"Couldn't read map file from " << color_map_path;
            abort();
        }

        sensor_msgs::PointCloud2 msg_map;
        pcl::toROSMsg(*color_cloud, msg_map);
        msg_map.header.stamp =  ros::Time(lidar_end_time_);
        msg_map.header.frame_id = frame_map_;
        pub_map_.publish(msg_map);
    }
    else {
        sensor_msgs::PointCloud2 msg_map;
        pcl::toROSMsg(*(p_map_->cloud_map), msg_map);
        msg_map.header.stamp =  ros::Time(lidar_end_time_);
        msg_map.header.frame_id = frame_map_;
        pub_map_.publish(msg_map);
        LOG(INFO) << "Publish map ! map size : " << p_map_->cloud_map->size();
        // p_map_->cloud_map.reset(new PointCloudXYZI());

    }



    // downsampleAndSavePointCloud(p_map_->cloud_map, "pcl_ds.pcd");
    bool b_first_run_ = true, b_flg_first_scan_ = true;

    bool b_need_init = true;
    LOG(INFO) << "START LOOP ! ";
    while (status)
    {
        ros::spinOnce();
        // if( b_first_run_ ) {
        //     imu_buffer_.clear();
        //     lidar_buffer_.clear();
        //     time_buffer_.clear();
        //     b_first_run_ = false;
        // }

        if (SyncPpackages(Measures_)){

            double start, end;
            start = omp_get_wtime();

            if (b_flg_first_scan_)  {


                if ( InitPose() ) {
                    p_map_->SetPose(gnss_state);
                }
                else{
                    LOG(WARNING) << "No GNSS data, cant init !!" ;
                    continue;
                }

                double time_s = omp_get_wtime();
                PointType point;
                point.x = gnss_state.pos(0), point.y= gnss_state.pos(1), point.z = gnss_state.pos(2);
                PointVector Storage;
                p_map_->ikdtree.Radius_Search(point, 100, Storage);
                LOG(INFO) << "Radius search cost time : " << (omp_get_wtime() - time_s)*1000 << " ms";


                // AddMapToViewer(p_map_->cloud_map, gnss_state.pos, vis);

                first_lidar_time = Measures_.lidar_bag_time;
                p_imu_->first_lidar_time = first_lidar_time;
                b_flg_first_scan_ = false;
                continue;
            }


            // std::cout<<"meas imu size : " << Measures_.imu.size()  << std::endl;
            p_imu_->Process(Measures_, p_map_->kf, cloud_frame_);


            cloud_frame_less_->clear();
            for( size_t i=0; i< cloud_frame_->size(); i++) 
                if (i %  point_filter_num_ == 0 ) 
                    cloud_frame_less_->emplace_back(cloud_frame_->points[i]);
// {
//             static int indx = 0;
//             if( !cloud_frame_less_->empty() ){
//                 std::cout<<"point num : "<< cloud_frame_->points.size() <<" less num : "<<cloud_frame_less_->points.size() <<std::endl;
//                 pcl::io::savePCDFileBinary("/home/tracer/workspace/code/c++/code2024/UGV/UGV2023_Green/program/module_modeling/bin/pcd_tmp1/" + to_string((lidar_end_time_*1000)) + ".pcd", *cloud_frame_);

//             }
// }


            LOG(INFO) << "points size ori : " <<cloud_frame_->size();

            if( p_map_->Process(cloud_frame_less_) ) {
                p_map_->GetPose(location_state);
                // AddFrameToViewer(p_map_->feats_down_world, location_state.pos, vis);
// {
//                 if( !p_map_->feats_down_world->empty() )
//                     pcl::io::savePCDFileBinary("/home/tracer/workspace/code/c++/code2024/UGV/UGV_lide/program/module_modeling/bin/pcd_tmp3/" + to_string((lidar_end_time_*1000)) + ".pcd", *(p_map_->feats_down_world));
// }

                CalLocationError();

                // ShowInSattle();
                PubLidarPose(location_state);
                PublishCloud(p_map_->ikdtree_cloud, p_map_->feats_down_world);

                pre_location_state = location_state;
            }

            end = omp_get_wtime();
            LOG(INFO) << "one loop cost time : " << (end - start)*1000 << "\n\n";
        }



        if (b_flg_exit_)     break;
        status = ros::ok();
        rate.sleep();
    }

}

void Localization::Run_handle()
{

    LOG(INFO) << "start run .";
    b_flg_exit_ = false;
    signal(SIGINT, Localization::SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();

    state_group gnss_state, location_state;

    double first_lidar_time;

    sensor_msgs::PointCloud2 msg_map;
    pcl::toROSMsg(*(p_map_->cloud_map), msg_map);
    msg_map.header.stamp =  ros::Time(lidar_end_time_);
    msg_map.header.frame_id = frame_map_;
    pub_map_.publish(msg_map);

    // downsampleAndSavePointCloud(p_map_->cloud_map, "pcl_ds.pcd");
    bool b_first_run_ = true, b_flg_first_scan_ = true;

    bool b_need_init = true;
    LOG(INFO) << "START LOOP ! ";
    while (status)
    {
        ros::spinOnce();

        if (SyncPpackages(Measures_)){

            double start, end;
            start = omp_get_wtime();

            if (b_flg_first_scan_)  {
                if( !gnss_buffer_.empty() ) 
                    gnss_state = GetInitStateFromFirstGNSS(gnss_buffer_.back());
                else{
                    LOG(WARNING) << "No GNSS data, cant init !!" ;
                    continue;
                }
                
  
                LOG(INFO) << "Before state : " << gnss_state.pos;
                GetHandInitPose(gnss_state, p_map_->cloud_map, Measures_.lidar);
                p_map_->SetPose(gnss_state);
                LOG(INFO) << "After state : " << gnss_state.pos;

                // AddMapToViewer(p_map_->cloud_map, gnss_state.pos, vis);

                first_lidar_time = Measures_.lidar_bag_time;
                p_imu_->first_lidar_time = first_lidar_time;
                b_flg_first_scan_ = false;
                continue;
            }

            p_imu_->Process(Measures_, p_map_->kf, cloud_frame_);

            cloud_frame_less_->clear();
            for( size_t i=0; i< cloud_frame_->size(); i++) 
                if (i %  point_filter_num_ == 0 ) 
                    cloud_frame_less_->emplace_back(cloud_frame_->points[i]);

            LOG(INFO) << "points size ori : " <<cloud_frame_->size();
        
            if( p_map_->Process(cloud_frame_less_) ) {
                p_map_->GetPose(location_state);
                // AddFrameToViewer(p_map_->feats_down_world, vis);
                PubLidarPose(location_state);
                PublishCloud(p_map_->cloud_map, p_map_->feats_down_world);


            }

            end = omp_get_wtime();
            LOG(INFO) << "one loop cost time : " << (end - start)*1000 << "\n\n";

        }



        if (b_flg_exit_)     break;
        status = ros::ok();
        rate.sleep();
    }

}






void Localization::Run_config()
{

    LOG(INFO) << "start run .";
    b_flg_exit_ = false;
    signal(SIGINT, Localization::SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();

    state_group gnss_state, location_state;


    // gnss_state = GetInitState(hand_pose_, delta_pose_ ,map_center_);
    gnss_state = GetInitState(hand_pose_ ,map_center_);

    double first_lidar_time;

    sensor_msgs::PointCloud2 msg_map;
    
    pcl::toROSMsg(*(p_map_->cloud_map), msg_map);

    msg_map.header.stamp =  ros::Time(lidar_end_time_);
    msg_map.header.frame_id = frame_map_;
    pub_map_.publish(msg_map);
    LOG(INFO) << "Publish map ! map size : " << p_map_->cloud_map->size();

    // downsampleAndSavePointCloud(p_map_->cloud_map, "pcl_ds.pcd");
    bool b_first_run_ = true, b_flg_first_scan_ = true;

    bool b_need_init = true;
    LOG(INFO) << "START LOOP ! ";
    while (status)
    {
        ros::spinOnce();

        if (SyncPpackages(Measures_)){

            double start, end;
            start = omp_get_wtime();

            if (b_flg_first_scan_)  {
                LOG(INFO) << "Before gnss_state :\n " << gnss_state.pos;
                GetAutoInitPose(gnss_state, p_map_->cloud_map, Measures_.lidar);
                p_map_->SetPose(gnss_state);
                LOG(INFO) << "After gnss_state :\n " << gnss_state.pos;

                AddMapToViewer(p_map_->cloud_map, gnss_state.pos, vis);

                first_lidar_time = Measures_.lidar_bag_time;
                p_imu_->first_lidar_time = first_lidar_time;
                b_flg_first_scan_ = false;
                continue;
            }


            p_imu_->Process(Measures_, p_map_->kf, cloud_frame_);


            cloud_frame_less_->clear();
            for( size_t i=0; i< cloud_frame_->size(); i++) 
                if (i %  point_filter_num_ == 0 ) 
                    cloud_frame_less_->emplace_back(cloud_frame_->points[i]);

            LOG(INFO) << "points size ori : " <<cloud_frame_->size();
        
            if( p_map_->Process(cloud_frame_less_) ) {
                p_map_->GetPose(location_state);
                AddFrameToViewer(p_map_->feats_down_world, vis);
            }
            
            end = omp_get_wtime();
            LOG(INFO) << "one loop cost time : " << (end - start)*1000 << "\n\n";


        }



        if (b_flg_exit_)     break;
        status = ros::ok();
        rate.sleep();
    }

}


void Localization::Run_fusion()
{
    b_flg_exit_ = false;
    signal(SIGINT, Localization::SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();

    state_group gnss_state, location_state;

    double first_lidar_time;

    bool b_first_run_ = true, b_flg_first_scan_ = true;

    bool b_need_init = true;
    LOG(INFO) << "START LOOP ! ";
    while (status)
    {
        ros::spinOnce();

        if (SyncPpackages(Measures_)){

            bool b_gnss_available = GetGNSSMeasure(gnss_state);


            if (b_need_init ) {
                if(b_gnss_available) {
                    p_map_->SetPose(gnss_state);
                    location_state = gnss_state;
                    b_need_init = false;
                } 
                else    
                    continue;
            } 

            
            bool b_relocation_needed = JudgeLocationState(gnss_state, location_state);
            if (b_gnss_available && b_relocation_needed ) 
                p_map_->SetPose(gnss_state);
            


            if (b_flg_first_scan_)  {
                first_lidar_time = Measures_.lidar_bag_time;
                p_imu_->first_lidar_time = first_lidar_time;
                b_flg_first_scan_ = false;
                continue;
            }


            p_imu_->Process(Measures_, p_map_->kf, cloud_frame_);


            cloud_frame_less_->clear();
            for( size_t i=0; i< cloud_frame_->size(); i++) 
                if (i %  point_filter_num_ == 0 ) 
                    cloud_frame_less_->emplace_back(cloud_frame_->points[i]);

            LOG(INFO) << "points size : " <<cloud_frame_->size();
        
            p_map_->Process(cloud_frame_less_);
            p_map_->GetPose(location_state);

        }



        if (b_flg_exit_)     break;
        status = ros::ok();
        rate.sleep();
    }

}


void Localization::Run()
{

    switch (init_method_)
    {
    case Gnss:
        Run_gnss();
        break;
    case Hand:
        Run_handle();
        break;
    case Config:
        Run_config();
        break;
    case Fusion:
        Run_fusion();
        break;    
    case Rviz:
        Run_rviz();
    default:
        break;
    }

}
