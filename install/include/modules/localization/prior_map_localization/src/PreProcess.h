#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "global_define.h"



class PreProcess
{
public:
    PreProcess();
    ~PreProcess();
    void Process(const LivoxPointCloud &cloud, PointCloudXYZI::Ptr &pcl_out);
    void Process(const pcl::PointCloud<velodyne_lidar::Point> &cloud,
                 PointCloudXYZI::Ptr &pcl_out);
    void Process(const pcl::PointCloud<rs_lidar::Point> &cloud,
                 PointCloudXYZI::Ptr &pcl_out);

    void Velodyne_handler(const pcl::PointCloud<velodyne_lidar::Point> &cloud);
    void Rs128_handler(const pcl::PointCloud<rs_lidar::Point> &cloud);
    void avia_handler(const LivoxPointCloud &cloud);
    void SetValidRegion(double x_min, double x_max,
                        double y_min, double y_max,
                        double z_min, double z_max,
                        double max_range);


//    yaml para
    bool feature_enabled;
    double blind;
    int lidar_type, N_SCANS, time_unit, SCAN_RATE, point_filter_num;



    float time_unit_scale;
    bool given_offset_time;


    PointCloudXYZI pl_full, pl_corn, pl_surf;

private:
    void UpdateTimeUnitScale();
    bool ValidPoint(double x, double y, double z) const;

    double box_x_min_ = -0.7;
    double box_x_max_ = 0.7;
    double box_y_min_ = -0.4;
    double box_y_max_ = 0.4;
    double box_z_min_ = -0.6;
    double box_z_max_ = 0.5;
    double max_range_squared_ = 15.0 * 15.0;

};

#endif // PREPROCESS_H
