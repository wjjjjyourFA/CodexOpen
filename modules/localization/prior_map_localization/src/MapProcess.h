#ifndef TEST_FAST_LIO_H
#define TEST_FAST_LIO_H


#include "global_define.h"
//#include "common_lib.h"
#include "ikd-Tree/ikd_Tree.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "self_state/GlobalPose.h"
#include "self_state/LidarLocalPose.h"

#include <csignal>
#include <unistd.h>




#include "PreProcess.h"
#include "ImuProcess.h"
using namespace std;

class MapProcess
{
public:
    MapProcess();
    ~MapProcess();

    bool Process(PointCloudXYZI::Ptr cloud_frame);
    void SetPose(state_group init_state);
    void GetPose(state_group &out_state);
    void SetMap(string map_path);

    bool InitConfig(const std::string &config_yaml);
    bool InitROS(ros::NodeHandle nh);
    void SetPcdDir(std::string path_pcd);
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
    void standard_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void livox_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void globalPose_cbk(const self_state::GlobalPose::ConstPtr &msg_in);
    void imuRawToRos(sensor_msgs::Imu::Ptr msg_in);


//    void GenetateMesureGroup();
    bool SyncPpackages(MeasureGroup &meas);
    void lasermap_fov_segment();
    void map_update();
    void map_incremental();
    inline static void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);


    void UpdateShowBuff(PointCloudXYZI::Ptr laserCloud_world_cur, PointCloudXYZI::Ptr laserCloud_world_save, bool &b_update_flag, int &count_save_pcd_num);
    void UpdateShowBuff(PointCloudXYZI::Ptr laserCloud_world_cur, bool &b_update_flag);
    void UpdateShowBuff2(PointCloudXYZI::Ptr laserCloud_world_map, PointCloudXYZI::Ptr laserCloud_world_cur);
    void UpdateShowBuff1(PointCloudXYZI::Ptr laserCloud_world_map, PointCloudXYZI::Ptr laserCloud_world_cur);

    void dump_lio_state_to_log(FILE *fp);




    void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s);
    void pointBodyToWorld(PointType const *const pi, PointType *const po);
    template <typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);
    void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
    void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
    void points_cache_collect();
    void ShowMap(PointCloudXYZI::Ptr laserCloud_world_map, PointCloudXYZI::Ptr laserCloud_world_cur);
    void GetWholeMap(PointCloudXYZI::Ptr cloud_map);


    string lid_topic, imu_topic, imuRaw_topic, lidarLocalPose_topic, globalPose_topic;  // 各话题名
    PointCloudXYZI::Ptr _featsArray;        // ikd-tree中，map需要移除的点云序列

    int invalid_frame_num = 0;

    V3D euler_rot_cur , euler_pos_cur;

    V3D lid_pos_cur;
    SO3 lid_rot_cur;


    double last_timestamp_lidar = 0;

    int featsFromMapNum ;

    pcl::PointXYZ cur_pose;
    pcl::PointXYZ pre_pose;













    // ros


    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu ;




//    ikdtree参数
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;     // ikdtree插入，搜索，删除耗时
    int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;// ikdtree 更新前，更新后，插入，删除个数

//    T1为雷达初始时间戳，s_plot为整个流程耗时，s_plot2特征点数量,s_plot3为kdtree增量时间，s_plot4为kdtree搜索耗时，s_plot5为kdtree删除点数量
//    s_plot6为kdtree删除耗时，s_plot7为kdtree初始大小，s_plot8为kdtree结束大小,s_plot9为平均消耗时间，s_plot10为添加点数量，s_plot11为点云预处理的总时间
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
    inline static double match_time = 0, solve_time = 0;  //匹配，求解耗时
    bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, path_en = true;     // 记录log，存pcd，在线时间同步，在线外参数估计，path话题发布
    inline static bool extrinsic_est_en = true;


    inline static float res_last[100000] = {0.0};   //残差，点到面距离平方和
    float DET_RANGE = 300.0f;           //设置的当前雷达系中心到各个地图边缘的距离
    const float MOV_THRESHOLD = 1.5f;   //设置的当前雷达系中心到各个地图边缘的权重
    double time_diff_lidar_to_imu = 0.0;    // lidar imu的时间误差

    int point_filter_num;
    mutex mtx_buffer;                   // lidar，imu数据的互斥锁
    condition_variable sig_buffer;      // 释放信号的条件变量

    string root_dir = ROOT_DIR;         // 根目录

    double last_timestamp_imu = -1.0;                     //
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_surf_min = 0, filter_size_map_min = 0;
    double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;

    int  time_log_counter = 0, scan_count = 0, publish_count = 0;
    inline static int effct_feat_num = 0, feats_down_size = 0;
    int NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;

    inline static bool point_selected_surf[100000] = {0};
    bool lidar_pushed = false,  flg_exit = false, flg_EKF_inited = true;
    bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false, path_pub_en = false;

    vector<BoxPointType> cub_needrm;        // ikd-tree中，地图需要移除的包围盒序列
    inline static vector<PointVector> Nearest_Points;     // 每个点的最近点序列

    vector<double> init_pos;                // 初始位置
    vector<double> init_rot;                // 初始姿态





    PointCloudXYZI::Ptr featsFromMap;           // ikdtree中保存的有效点，l系
    PointCloudXYZI::Ptr feats_undistort;        // 去畸变后的特征点,l系
    PointCloudXYZI::Ptr feats_undistort_world;        // 去畸变后的特征点,w系
    PointCloudXYZI::Ptr feats_undistort_full;

    inline static PointCloudXYZI::Ptr cloud_map;        // 去畸变并降采样后的特征点,l系

    inline static PointCloudXYZI::Ptr feats_down_body;        // 去畸变并降采样后的特征点,l系
    inline static PointCloudXYZI::Ptr feats_down_world;       // 去畸变并降采样后的特征点,w系
    
    inline static PointCloudXYZI::Ptr normvec;                // 计算点-面残差时，存储对应平面的参数，但稀疏结构有空值，w系
    inline static PointCloudXYZI::Ptr laserCloudOri;          // 计算点-面残差时，实际用到的满足要求的点坐标，l系
    inline static PointCloudXYZI::Ptr corr_normvect;          // 计算点-面残差时，实际用到的点对应平面的参数，w系
    PointCloudXYZI::Ptr pcl_wait_save;          // 等待存储至本地的点云
//    PointCloudXYZI::Ptr laserCloudWorld;        // ui显示的当前帧点云
//    PointCloudXYZI::Ptr laserCloudWorld_sum;    // ui显示的所有帧点云


    pcl::VoxelGrid<PointType> downSizeFilterSurf;   // 特征点下采样器
    pcl::VoxelGrid<PointType> downSizeFilterMap;    // 地图点下采样器

    inline static KD_TREE<PointType> ikdtree;                 // 存储地图的ikdtree
//    inline static KD_TREE ikdtree;                 // 存储地图的ikdtree


    V3D Lidar_T_wrt_IMU = Eigen::Vector3d::Zero();      // lidar外参 t_i_l
    M3D Lidar_R_wrt_IMU = Eigen::Matrix3d::Zero();      // lidar外参 R_i_l




//    /*** EKF inputs and output ***/
    MeasureGroup Measures;                              // 组合的测量数据
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;    // eskf滤波器
    state_ikfom state_point;                            // imu坐标状态（位置，姿态，外参，速度，偏置，重力）
    vect3 pos_lid;                                      // lidar位置

    nav_msgs::Path path;                                // ros发布的path，一个vector
    nav_msgs::Odometry odomAftMapped;                   // ros发布的odom
    geometry_msgs::PoseStamped msg_body_pose;           // path中的一个点

    self_state::GlobalPose cur_globalPose;
    self_state::LidarLocalPose lidarLocalPose;




    Eigen::Matrix4f transform_mat;          // 变换矩阵，Tr_rfu_flu

    FILE *fp;                               // 打印LIO状态信息  ，与fout_out类似
    ofstream fout_pre, fout_out;            // 分别打印更新前与更新后LIO状态信息

    bool using_ins_init = true;             // 是否使用全局初始化






    int  frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;

    string my_log_dir = root_dir + "/Log";      // Log文件存放位置
    string my_map_dir = root_dir + "/Map";      // Map文件存放位置
    string my_pcd_dir = root_dir + "/Pcd";      // Map文件存放位置


    double timediff_lidar_wrt_imu = 0.0;        // 估计的lidar_imu之间的时间误差
    bool timediff_set_flg = false;              // 估计lidar_imu时间差是否完成标志位
    double lidar_mean_scantime = 0.0;           // lidar末尾点的平均时间
    int scan_num = 0;                           // lidar帧数

    BoxPointType LocalMap_Points;               // ikdtree地图边界

    std::string ros_bag_name ;



    std::mutex lock_flg;

    int points_num_ = 0;



    pcl::visualization::PCLVisualizer *vis ;

    PointCloudXYZI::Ptr ikdtree_cloud;
//    回环检测


};

#endif
