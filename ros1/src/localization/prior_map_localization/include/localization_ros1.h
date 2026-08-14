
/**
 * @file Localization.h
 * @author Bokai ()
 * @brief 
 * @version 0.1
 * @date 2024-04-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <iostream>
#include <deque>
#include <fstream>
#include <filesystem>

#include <mutex>
#include <omp.h>

#include <glog/logging.h>


#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>

#include "livox_ros_driver2/CustomMsg.h"
#include "livox_ros_driver2/CustomPoint.h"

#include "self_state/GlobalPose.h"
#include "self_state/LocalPose.h"
#include "self_state/LidarGlobalPose.h"

#include "global_define.h"
#include "utils/Colors_utils.hpp"
#include "src/ImuProcess.h"
#include "src/PreProcess.h"
#include "src/MapProcess.h"

#include "CoordOur.hpp"
#include "utils/Sattle_util.hpp"

#include <sensor_msgs/CompressedImage.h>
using namespace std;



class Localization 
{
public:
    Localization(ros::NodeHandle p_node,
                 const std::string& runtime_config,
                 const std::string& map_path,
                 const std::string& log_path);
    ~Localization();

    void ReadTopic();
    void ReadConfig();
    bool Init();
    
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
    void raw_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void livox_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void globalPose_cbk(const self_state::GlobalPose::ConstPtr &msg_in);
    void localPose_cbk(const self_state::LocalPose::ConstPtr &msg_in);
    void imuRawToRos(const sensor_msgs::Imu::ConstPtr &msg_raw, sensor_msgs::Imu::Ptr &msg);
    void rvizPose_cbk(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void PubLidarPose(state_group location_state);
    void PublishCloud(PointCloudXYZI::Ptr map, PointCloudXYZI::Ptr frame);
    bool JudgeLocationState(state_group gnss_state, state_group location_state);
    bool GetGNSSMeasure(state_group &gnss_state);


    bool InitPose();
    bool SyncPpackages(MeasureGroup &meas);
    void Run();
    void Run_handle();
    void Run_gnss();
    void Run_config();
    void Run_fusion();
    void Run_rviz();

    void GetHandInitPose(state_group &init_state,PointCloudXYZI::Ptr map, PointCloudXYZI::Ptr frame);
    void GetAutoInitPose(state_group &init_state,PointCloudXYZI::Ptr map, PointCloudXYZI::Ptr frame);

    void ShowInitResult(pcl::visualization::PCLVisualizer::Ptr vis, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map,
                        string window_name );
    void ShowMatchResultDual(pcl::visualization::PCLVisualizer::Ptr vis, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map,
                        string window_name );

    void AddMapToViewer(PointCloudXYZI::Ptr map, Eigen::Vector3d pos, pcl::visualization::PCLVisualizer::Ptr viewer);
    void AddFrameToViewer(PointCloudXYZI::Ptr frame, Eigen::Vector3d pos , pcl::visualization::PCLVisualizer::Ptr viewer);
    void AddFrameToViewer(PointCloudXYZI::Ptr frame,  pcl::visualization::PCLVisualizer::Ptr viewer);
    void CalLocationError();

    void PubOdometry(state_group location_state);
    static void SigHandle(int sig) {
        b_flg_exit_ = true;
        ROS_WARN("catch sig %d", sig);
        sig_buffer_.notify_all();
    }
    void ShowInSattle();
    cv::Mat sattle_mat;
    cv::Mat intensity_mat;


    shared_ptr<PreProcess> p_pre_ = nullptr;
    shared_ptr<ImuProcess> p_imu_ = nullptr;
    shared_ptr<MapProcess> p_map_ = nullptr;
    // shared_ptr<Viewer> p_ui_ = nullptr;      // ui对象

    // std::thread render_thread_;                 // ui线程
    // std::mutex mutex_ui_;           // 可视化buffer互斥锁

    PointCloudXYZI::Ptr cloud_frame_;        // 去畸变后的特征点,l系
    PointCloudXYZI::Ptr cloud_frame_less_;        // 去畸变后的特征点,l系



    ros::NodeHandle p_node_;

    string config_yaml_;
    string map_path_override_;
    string log_path_override_;

    string topic_lidar_;
    string topic_imu_;
    string topic_lidar_livox_;
    string topic_imu_livox_;
    string topic_global_pose_;
    string topic_local_pose_;
    string topic_lidar_pose_;
    string topic_initial_pose_;
    string topic_map_;
    string topic_path_;
    string topic_lio_odom_;
    string topic_lio_path_;
    string topic_state_estimation_;
    string topic_registered_scan_;

    string frame_map_;
    string frame_world_;
    string frame_sensor_;
    string frame_body_;

    int lidar_queue_size_;
    int imu_queue_size_;
    int initial_pose_queue_size_;
    int map_queue_size_;
    int path_queue_size_;
    int odometry_queue_size_;
    int lio_path_queue_size_;
    int registered_scan_queue_size_;
    bool map_latched_;

    ros::Publisher pub_lidarPose_;
    ros::Publisher pub_map_;
    ros::Publisher pub_frame_;
    ros::Publisher pub_path_;
    ros::Publisher pub_lio_odom_;
    ros::Publisher pub_lio_path_;
    ros::Publisher pub_fakeGPose;
    ros::Publisher pub_result_mat;

    string topic_fakeGPose = "/self_state/GlobalPoseLidar";

    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_global_pose_;
    ros::Subscriber sub_local_pose_;
    ros::Subscriber sub_rviz_pose_;
    static bool b_flg_exit_;
    static condition_variable sig_buffer_;
    std::mutex mtx_buffer_;

    // static bool b_flg_exit_;
    bool b_save_log_ = false,  b_publish_ros_ = true , b_with_ui_ = false;
    bool b_display_init_ = false, b_twice_init_ = true;
    string path_log_, save_path_, path_load_map_;
    int lidar_type_;
    ofstream fout_log_pose_;            // 输出gnss pose
    ofstream fout_log_time_;      // 输出gnss center

    int scan_count_;
    double last_timestamp_lidar_, last_timestamp_imu_, last_timestamp_gp_, last_timestamp_lp_;
    double lidar_end_time_, lidar_bag_time_;

    MeasureGroup Measures_;                              // 组合的测量数据
    bool b_lidar_pushed_ = false;
    deque<double> time_buffer_;                      // 电云时间戳buff
    deque<PointCloudXYZI::Ptr> lidar_buffer_;        // 点云buff
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;   // imu数据buff
    deque<self_state::GlobalPose::Ptr> gp_buffer_;
    deque<self_state::LocalPose::Ptr> lp_buffer_;
    deque<nav_msgs::Odometry::Ptr> gnss_buffer_;

    vector<double> map_center_;

    int point_filter_num_;
    double threshold_position_, threshold_rotation_;

    vector<double> hand_pose_;
    vector<double> delta_pose_;

    int init_method_;
    pcl::visualization::PCLVisualizer::Ptr vis;

    nav_msgs::Path path_vec_;
    nav_msgs::Path lio_path_vec_;
    state_group gnss_state, location_state, pre_location_state;

    double lidar_mean_scantime = 0.0;           // lidar末尾点的平均时间
    int scan_num = 0;                           // lidar帧数

    ofstream fout_error, fout_loc, fout_gnss;


    // Sattle_util sattle_utiler;


    // rviz
    geometry_msgs::PoseWithCovarianceStamped init_pose_msg_;
    bool init_pose_received_ = false;



    tf::TransformBroadcaster tfBroadcaster;
    ros::Publisher pub_odometry;

    float z_offset_;

};
