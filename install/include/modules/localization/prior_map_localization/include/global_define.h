#ifndef GLOBAL_DEFINE_H
#define GLOBAL_DEFINE_H


// system
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <time.h>
#include <vector>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>


// pangolin
//#include <pangolin/pangolin.h>


// yaml-cpp
#include <yaml-cpp/yaml.h>

// pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

// opencv
//#include "opencv2/opencv.hpp"


#include <omp.h>

// GLOG
#include <glog/logging.h>
#include <gflags/gflags.h>



#include "common_lib.h"
#include "use-ikfom.hpp"
using namespace  std;

#define roi_range 100


#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)


// 宏定义
#define     DEG_2_RAD   0.017453293      // M_PI / 180.0
#define     RAD_2_DEG   57.29578         // 180.0 / M_PI
#define     BASE_X      19695752.27
#define		BASE_Y      3125228.07


enum INIT_METHOD{Gnss = 1, Hand, Config, Fusion, Rviz};

enum ODOM_METHOD{Odom_FastLIO = 1, Odom_ICP, Odom_NDT};   //{1, 2, 3}

enum LOOP_METHOD{Loop_GNSS = 1, Loop_SC, Loop_Bow};





enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};


enum LID_TYPE{AVIA = 1, RS128, RSM1, RS32, HS128, VELO16}; //{1, 2, 3}


namespace my_file {
static bool get_filelist_from_dir(std::string _path, std::vector<std::string> &_files)
{
    DIR *dir;
    dir = opendir(_path.c_str());

    if (dir == NULL)
    {
        printf("d == NULL");
        return false;
    }
    struct dirent *ptr;
    std::vector<std::string> file;
    while ((ptr = readdir(dir)) != NULL)
    {
        if (ptr->d_name[0] == '.')
            continue;

        file.push_back(ptr->d_name);
    }
    closedir(dir);
//    sort(file.begin(), file.end());

    sort(file.begin(), file.end(), [](string a, string b){
        return stoi(a) < stoi(b);
    });

    // // 打印排序后的文件名
    // for (const auto& file_name : file) {
    //     std::cout << file_name << std::endl;
    // }

    _files = file;

    return true;
}
static bool is_exists(const std::string &name) {
    std::ifstream f(name.c_str());
    return f.good();
}

}


namespace rs_lidar {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rs_lidar::Point,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (uint8_t, intensity, intensity)
                                    (uint16_t, ring, ring)
                                    (double, timestamp, timestamp)
)

namespace velodyne_lidar {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_lidar
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_lidar::Point,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, time, time)
                                    (uint16_t, ring, ring))

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                    (float, x, x) (float, y, y)
                                    (float, z, z) (float, intensity, intensity)
                                    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                    (double, time, time))

typedef PointXYZIRPYT  PointTypePose;




struct Pose_txyzrpy {
    double time;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};


struct state_pose {
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
//    V3D pos;
//    SO3 rot;
};

inline bool ReadPose( std::string path_pose, std::map<double, state_pose>& pose_map )
{
    std::ifstream file(path_pose);

    if (!file.is_open()) {
        std::cerr << "Failed to open " << path_pose << std::endl;
        return false;
    }

    double timestamp, x, y, z, qx, qy, qz, qw;

    while (file >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw) {
        state_pose pose;
        pose.pos << x, y, z;
        pose.rot = Eigen::Quaterniond(qw, qx, qy, qz);
        pose_map[timestamp] = pose;
//        pose_map.insert(std::make_pair(timestamp, pose));
    }
    LOG(INFO) <<"pose path : "<<path_pose<<endl;
    LOG(INFO) <<"pose size : "<<pose_map.size()<<endl;
    file.close();
    return true;
}

inline void AccumeCloud(std::string path_pcd, std::map<double, state_pose> pose_map, PointCloudXYZI::Ptr& cloud)
{
    std::vector<PointCloudXYZI::Ptr> cloud_vec;
    PointCloudXYZI::Ptr cloud_cur(new PointCloudXYZI());
    PointCloudXYZI::Ptr cloud_cur_world(new PointCloudXYZI());

    PointCloudXYZI::Ptr cloud_sum(new PointCloudXYZI());

    vector<string> pcd_vector;
    my_file::get_filelist_from_dir(path_pcd + "/", pcd_vector);
    if( pcd_vector.size() == 0 )
        LOG(ERROR) << "no sub pcd file for save ." ;



    LOG(INFO) << "pcd size : " << pcd_vector.size()<<"  pose size : "<<pose_map.size();

    assert(pcd_vector.size() == pose_map.size());




    int frame_idx=0;
    for( auto map_iter=pose_map.begin(); map_iter!=pose_map.end(); ++map_iter ) {
        string file_lidar = path_pcd + "/" + string(pcd_vector[frame_idx]);
        pcl::io::loadPCDFile(file_lidar, *cloud_cur);



        Eigen::Affine3d Tr = Eigen::Affine3d::Identity();
        Tr.translation() = map_iter->second.pos;
        Tr.rotate(map_iter->second.rot.matrix());

        pcl::transformPointCloud(*cloud_cur, *cloud_cur, Tr);
        *cloud_sum = *cloud_sum + *cloud_cur;

        if( frame_idx%100==0 ) {
            PointCloudXYZI::Ptr cloud_sum_tmp(new PointCloudXYZI());
            cloud_sum.swap(cloud_sum_tmp);
            cloud_vec.emplace_back(cloud_sum_tmp);
        }
        LOG(INFO) << "Load pcd num, map num  "<<frame_idx+1<<"/"<<pcd_vector.size() << "  " << cloud_cur->size()<<" "<<cloud_sum->size();


        frame_idx++;
    }

    PointCloudXYZI::Ptr cloud_sum_tmp(new PointCloudXYZI());
    cloud_sum.swap(cloud_sum_tmp);
    cloud_vec.emplace_back(cloud_sum_tmp);


    for( size_t i=0; i<cloud_vec.size(); i++ ) {
        *cloud_sum += *(cloud_vec[i]);
    }
    cloud_vec.clear();
    cloud = cloud_sum;
}


inline void AccumeCloudDS(std::string path_pcd, std::map<double, state_pose> pose_map, PointCloudXYZI::Ptr& cloud, double filter_size=0.1)
{

    pcl::VoxelGrid<PointType> downSizeFilterMap;    // 地图点下采样器
    downSizeFilterMap.setLeafSize(filter_size, filter_size, filter_size);


    std::vector<PointCloudXYZI::Ptr> cloud_vec;
    PointCloudXYZI::Ptr cloud_cur(new PointCloudXYZI());
    PointCloudXYZI::Ptr cloud_cur_world(new PointCloudXYZI());

    PointCloudXYZI::Ptr cloud_sum(new PointCloudXYZI());

    vector<string> pcd_vector;
    my_file::get_filelist_from_dir(path_pcd + "/", pcd_vector);
    if( pcd_vector.size() == 0 )
        LOG(ERROR) << "no sub pcd file for save ." ;



    LOG(INFO) << "pcd size : " << pcd_vector.size()<<"  pose size : "<<pose_map.size();

    assert(pcd_vector.size() == pose_map.size());




    int frame_idx=0;
    for( auto map_iter=pose_map.begin(); map_iter!=pose_map.end(); ++map_iter ) {
        string file_lidar = path_pcd + "/" + string(pcd_vector[frame_idx]);
        pcl::io::loadPCDFile(file_lidar, *cloud_cur);

        downSizeFilterMap.setInputCloud(cloud_cur);
        downSizeFilterMap.filter(*cloud_cur);

        Eigen::Affine3d Tr = Eigen::Affine3d::Identity();
        Tr.translation() = map_iter->second.pos;
        Tr.rotate(map_iter->second.rot.matrix());

        pcl::transformPointCloud(*cloud_cur, *cloud_cur, Tr);
        *cloud_sum = *cloud_sum + *cloud_cur;

        if( frame_idx%100==0 ) {
            PointCloudXYZI::Ptr cloud_sum_tmp(new PointCloudXYZI());
            cloud_sum.swap(cloud_sum_tmp);

            downSizeFilterMap.setInputCloud(cloud_sum_tmp);
            downSizeFilterMap.filter(*cloud_sum_tmp);

            cloud_vec.emplace_back(cloud_sum_tmp);
        }
        LOG(INFO) << "Load pcd num, map num  "<<frame_idx+1<<"/"<<pcd_vector.size() << "  " << cloud_cur->size()<<" "<<cloud_sum->size();


        frame_idx++;
    }

    PointCloudXYZI::Ptr cloud_sum_tmp(new PointCloudXYZI());
    cloud_sum.swap(cloud_sum_tmp);
    cloud_vec.emplace_back(cloud_sum_tmp);


    for( size_t i=0; i<cloud_vec.size(); i++ ) {
        *cloud_sum += *(cloud_vec[i]);
    }
    cloud_vec.clear();
    cloud = cloud_sum;
}


namespace Trans  {

static state_pose OurEulerToState(Pose_txyzrpy &pos) {
    state_pose result;
    Eigen::Affine3d affine;
    pcl::getTransformation(pos.x, pos.y, pos.z, pos.roll, pos.pitch, pos.yaw, affine);

    result.rot =  affine.rotation();
    result.pos = affine.translation();
    return result;
}

};


inline Eigen::Quaterniond EulerToSO3(float roll_, float pitch_, float yaw_)
{
    Eigen::Quaterniond q; //   四元数 q 和 -q 是相等的
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll;
    q.normalize();
    return q;
}



template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

#endif
