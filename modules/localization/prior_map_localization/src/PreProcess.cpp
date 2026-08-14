
#include "PreProcess.h"

#include <cmath>

PreProcess::PreProcess()
{

}

PreProcess::~PreProcess()
{

}

void PreProcess::SetValidRegion(double x_min, double x_max,
                                double y_min, double y_max,
                                double z_min, double z_max,
                                double max_range)
{
    box_x_min_ = x_min;
    box_x_max_ = x_max;
    box_y_min_ = y_min;
    box_y_max_ = y_max;
    box_z_min_ = z_min;
    box_z_max_ = z_max;
    max_range_squared_ = max_range * max_range;
}

bool PreProcess::ValidPoint(double x, double y, double z) const
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
        return false;
    }

    const double distance_squared = x * x + y * y + z * z;
    if (distance_squared >= max_range_squared_)
    {
        return false;
    }

    const bool inside_robot_body =
        x > box_x_min_ && x < box_x_max_ &&
        y > box_y_min_ && y < box_y_max_ &&
        z > box_z_min_ && z < box_z_max_;
    return !inside_robot_body;
}

void PreProcess::Process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
    avia_handler(msg);
    *pcl_out = pl_surf;
}

void PreProcess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{

    switch (time_unit)
    {
    case SEC:
        time_unit_scale = 1.e3f;
        break;
    case MS:
        time_unit_scale = 1.f;
        break;
    case US:
        time_unit_scale = 1.e-3f;
        break;
    case NS:
        time_unit_scale = 1.e-6f;
        break;
    default:
        time_unit_scale = 1.f;
        break;
    }



    switch (lidar_type)
    {
    case VELO16:
        Velodyne_handler(msg);
        break;

    case RS128:
        Rs128_handler(msg);
        break;



    default:
        LOG(ERROR) << " Error LiDAR Type : "<< lidar_type;
        break;
    }
    *pcl_out = pl_surf;
}

void PreProcess::avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    double t1 = omp_get_wtime();
    int plsize = msg->point_num;



    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    uint valid_num = 0;


    for(uint i=1; i<plsize; i++)
    {
        if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num ++;
            {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

                if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
                        || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
                        || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
                        && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind))
                        && ValidPoint(pl_full[i].x, pl_full[i].y, pl_full[i].z))
                {
                    pl_surf.push_back(pl_full[i]);
                }

            }
        }
    }



}

void PreProcess::Velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
        given_offset_time = true;
    }
    else
    {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end  = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--)
        {
            if (pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }



    for (int i = 0; i < plsize; i++)
    {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time)
        {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer])
            {
                // printf("layer: %d; is first: %d", layer, is_first[layer]);
                yaw_fp[layer]=yaw_angle;
                is_first[layer]=false;
                added_pt.curvature = 0.0;
                yaw_last[layer]=yaw_angle;
                time_last[layer]=added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer])
            {
                added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
            }
            else
            {
                added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
            if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind)
                && ValidPoint(added_pt.x, added_pt.y, added_pt.z))
            {
                pl_surf.points.push_back(added_pt);
            }
        }
    }

}

void PreProcess::Rs128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_full.clear();

    // double time_scan =  msg->header.stamp.toSec();

    pcl::PointCloud<rs_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    // double time_scan = pl_orig.points.front().timestamp;

    auto min_point_it = std::min_element(pl_orig.begin(), pl_orig.end(),
    [](const auto& a, const auto& b) {
        return a.timestamp < b.timestamp;
    });

    auto max_point_it = std::max_element(pl_orig.begin(), pl_orig.end(),
    [](const auto& a, const auto& b) {
        return a.timestamp < b.timestamp;
    });

    double time_scan = min_point_it->timestamp;
    double time_start = min_point_it->timestamp;
    double time_end = max_point_it->timestamp;
    double time_diff = time_end - time_start;

    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].timestamp > 0)
    {
        given_offset_time = true;
    }
    else
    {
        ROS_WARN("rs128 : timestamp field is invalid .");
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end  = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--)
        {
            if (pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }



    for (int i = 0; i < plsize; i++)
    {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        // added_pt.curvature = (pl_orig.points[i].timestamp - time_scan)  * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;
        added_pt.curvature = (pl_orig.points[i].timestamp - time_start)/time_diff*100 ; // units: ms

        if (!given_offset_time)
        {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer])
            {
                // printf("layer: %d; is first: %d", layer, is_first[layer]);
                yaw_fp[layer]=yaw_angle;
                is_first[layer]=false;
                added_pt.curvature = 0.0;
                yaw_last[layer]=yaw_angle;
                time_last[layer]=added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer])
            {
                added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
            }
            else
            {
                added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer]=added_pt.curvature;
        }


        // if (i % point_filter_num == 0)
        {
            float range = added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z;
            // if(range > (blind * blind) && range < roi_range*roi_range)
            if(range > (blind * blind)
                && ValidPoint(added_pt.x, added_pt.y, added_pt.z))
            {
                pl_surf.points.push_back(added_pt);
            }
        }

    }

}
