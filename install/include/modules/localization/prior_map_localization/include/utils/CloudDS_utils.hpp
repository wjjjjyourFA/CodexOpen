#ifndef CLOUDDS_UTILS_HPP
#define CLOUDDS_UTILS_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_map>

using namespace std;

#define HASH_P 116101
#define MAX_N 10000000000

class VOXEL_LOC
{
public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
};


namespace std
{
template <>
struct hash<VOXEL_LOC>
{
    size_t operator()(const VOXEL_LOC &s) const
    {
        using std::size_t;
        using std::hash;
        return (((hash<int64_t>()(s.z) * HASH_P) % MAX_N + hash<int64_t>()(s.y)) * HASH_P) % MAX_N + hash<int64_t>()(s.x);
    }
};

struct M_POINT
{
    float xyz[3];
    float rgb[3];
    uint8_t intensity;

    int count = 0;
};


template <typename PointT>
inline void VoxelGridDownsamplePCL(typename pcl::PointCloud<PointT>::Ptr src, typename pcl::PointCloud<PointT>::Ptr &dst, double leaf_size)
{
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(src);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*dst);
}


template <typename PointT>
inline void VoxelGridLargeScaleXYZ(typename pcl::PointCloud<PointT>::Ptr src, typename pcl::PointCloud<PointT>::Ptr &dst, double leaf_size)
{
    typename pcl::PointCloud<PointT>::Ptr ptr_voxelized(new pcl::PointCloud<PointT>);
    std::unordered_map<VOXEL_LOC, M_POINT> feature_map;
    size_t pt_size = src->size();

    for (size_t i = 0; i < pt_size; i++)
    {
        PointT &pt_trans = src->points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++)
        {
            loc_xyz[j] = pt_trans.data[j] / leaf_size;
            if (loc_xyz[j] < 0)
                loc_xyz[j] -= 1.0;
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feature_map.find(position);
        if (iter != feature_map.end())
        {
            iter->second.xyz[0] += pt_trans.x;
            iter->second.xyz[1] += pt_trans.y;
            iter->second.xyz[2] += pt_trans.z;
            iter->second.count++;
        }
        else
        {
            M_POINT anp;
            anp.xyz[0] = pt_trans.x;
            anp.xyz[1] = pt_trans.y;
            anp.xyz[2] = pt_trans.z;
            anp.count = 1;
            feature_map[position] = anp;
        }
    }

    pt_size = feature_map.size();
    ptr_voxelized->clear();
    ptr_voxelized->resize(pt_size);

    size_t i = 0;
    for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter)
    {
        ptr_voxelized->points[i].x = iter->second.xyz[0] / iter->second.count;
        ptr_voxelized->points[i].y = iter->second.xyz[1] / iter->second.count;
        ptr_voxelized->points[i].z = iter->second.xyz[2] / iter->second.count;
        i++;
    }

    dst = ptr_voxelized;
}

template <typename PointT>
inline void VoxelGridLargeScaleIntensity(typename pcl::PointCloud<PointT>::Ptr src, typename pcl::PointCloud<PointT>::Ptr &dst, double leaf_size)
{
    typename pcl::PointCloud<PointT>::Ptr ptr_voxelized(new pcl::PointCloud<PointT>);
    std::unordered_map<VOXEL_LOC, M_POINT> feature_map;
    size_t pt_size = src->size();

    for (size_t i = 0; i < pt_size; i++)
    {
        PointT &pt_trans = src->points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++)
        {
            loc_xyz[j] = pt_trans.data[j] / leaf_size;
            if (loc_xyz[j] < 0)
                loc_xyz[j] -= 1.0;
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feature_map.find(position);
        if (iter != feature_map.end())
        {
            iter->second.xyz[0] += pt_trans.x;
            iter->second.xyz[1] += pt_trans.y;
            iter->second.xyz[2] += pt_trans.z;
            iter->second.intensity += pt_trans.intensity;
            iter->second.count++;
        }
        else
        {
            M_POINT anp;
            anp.xyz[0] = pt_trans.x;
            anp.xyz[1] = pt_trans.y;
            anp.xyz[2] = pt_trans.z;
            anp.intensity = pt_trans.intensity;
            anp.count = 1;
            feature_map[position] = anp;
        }
    }

    pt_size = feature_map.size();
    ptr_voxelized->clear();
    ptr_voxelized->resize(pt_size);

    size_t i = 0;
    for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter)
    {
        ptr_voxelized->points[i].x = iter->second.xyz[0] / iter->second.count;
        ptr_voxelized->points[i].y = iter->second.xyz[1] / iter->second.count;
        ptr_voxelized->points[i].z = iter->second.xyz[2] / iter->second.count;
        ptr_voxelized->points[i].intensity = iter->second.intensity / iter->second.count;
        i++;
    }

    dst = ptr_voxelized;
}

inline void VoxelGridLargeScaleRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst, double leaf_size)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::unordered_map<VOXEL_LOC, M_POINT> feature_map;
    size_t pt_size = src->size();

    for (size_t i = 0; i < pt_size; i++)
    {
        pcl::PointXYZRGB &pt_trans = src->points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++)
        {
            loc_xyz[j] = pt_trans.data[j] / leaf_size;
            if (loc_xyz[j] < 0)
                loc_xyz[j] -= 1.0;
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feature_map.find(position);
        if (iter != feature_map.end())
        {
            iter->second.xyz[0] += pt_trans.x;
            iter->second.xyz[1] += pt_trans.y;
            iter->second.xyz[2] += pt_trans.z;
            iter->second.rgb[0] += pt_trans.r;
            iter->second.rgb[1] += pt_trans.g;
            iter->second.rgb[2] += pt_trans.b;
            iter->second.count++;
        }
        else
        {
            M_POINT anp;
            anp.xyz[0] = pt_trans.x;
            anp.xyz[1] = pt_trans.y;
            anp.xyz[2] = pt_trans.z;
            anp.rgb[0] = pt_trans.r;
            anp.rgb[1] = pt_trans.g;
            anp.rgb[2] = pt_trans.b;
            anp.count = 1;
            feature_map[position] = anp;
        }
    }

    pt_size = feature_map.size();
    ptr_voxelized->clear();
    ptr_voxelized->resize(pt_size);

    size_t i = 0;
    for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter)
    {
        ptr_voxelized->points[i].x = iter->second.xyz[0] / iter->second.count;
        ptr_voxelized->points[i].y = iter->second.xyz[1] / iter->second.count;
        ptr_voxelized->points[i].z = iter->second.xyz[2] / iter->second.count;
        ptr_voxelized->points[i].r = iter->second.rgb[0] / iter->second.count;
        ptr_voxelized->points[i].g = iter->second.rgb[1] / iter->second.count;
        ptr_voxelized->points[i].b = iter->second.rgb[2] / iter->second.count;
        i++;
    }

    dst = ptr_voxelized;
}




} // namespace std

#endif // CLOUDDS_UTILS_HPP

