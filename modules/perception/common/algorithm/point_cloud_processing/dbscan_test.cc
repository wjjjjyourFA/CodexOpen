#include "modules/perception/common/algorithm/point_cloud_processing/dbscan.h"
#include <modules/perception/common/base/point.h>

#include "gtest/gtest.h"

#include "glog/logging.h"

using namespace jojo::perception::algorithm;
using namespace jojo::perception::base;

TEST(DbscanTest, init_and_cluster) {
  std::vector<Point3DF> points = {
      {1.0, 1.0, 1.0}, {1.5, 1.5, 1.5}, {0.5, 0.5, 1.5}, {2.0, 2.0, 2.0},
      {2.0, 2.0, 3.0}, {8.0, 8.0, 1.0}, {8.5, 8.5, 2.0}, {9.0, 9.0, 8.0}};
  std::vector<PointF> points_v = {{1.0, 1.0, 1.0, 1.0}, {1.5, 1.5, 1.5, 1.0},
                                  {0.5, 0.5, 1.5, 1.0}, {2.0, 2.0, 2.0, 1.0},
                                  {2.0, 2.0, 3.0, 1.0}, {8.0, 8.0, 1.0, 1.0},
                                  {8.5, 8.5, 2.0, 2.0}, {9.0, 9.0, 8.0, 1.0}};

  double eps = 1.5;
  int minPts = 2;

  DBSCAN<Point3DF> dbscan1;
  dbscan1.set_params(eps, minPts);
  dbscan1.SetInputCloud(points);
  dbscan1.Run();

  std::vector<int> labels1 = dbscan1.GetLabels();
  for (size_t i = 0; i < labels1.size(); ++i) {
    std::cout << "Point " << i << " -> Cluster " << labels1[i] << "\n";
  }

  DBSCAN<PointF> dbscan2;
  dbscan2.set_params(eps, minPts);
  dbscan2.SetInputCloud(points_v);
  dbscan2.Run();

  std::vector<int> labels2 = dbscan2.GetLabels();
  for (size_t i = 0; i < labels2.size(); ++i) {
    std::cout << "Point " << i << " -> Cluster " << labels2[i] << "\n";
  }
};