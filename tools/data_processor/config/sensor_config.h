#ifndef __CONFIG_H
#define __CONFIG_H

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <semaphore.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std;

extern std::map<string, int> CameraTypeParam;

extern std::map<string, int> RadarTypeParam;

extern std::map<string, int> Radar4DTypeParam;

extern std::map<string, int> LidarTypeParam;
#endif  // __CONFIG_H
