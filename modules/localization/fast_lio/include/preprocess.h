#ifndef PREPROCESS_H
#define PREPROCESS_H

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

// typedef pcl::PointXYZINormal PointType;
// typedef pcl::PointCloud<PointType> PointCloudXYZI;

// enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM}; // {1, 2, 3}
enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM, RS128, RSM1, RS32, HS128};
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

#endif