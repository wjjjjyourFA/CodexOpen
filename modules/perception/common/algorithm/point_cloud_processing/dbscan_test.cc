#include "modules/perception/common/algorithm/point_cloud_processing/dbscan.h"
#include "modules/perception/common/algorithm/point_cloud_processing/dbscan_extensions.h"

#include <algorithm>
#include <vector>

#include <modules/perception/common/base/point.h>

#include "gtest/gtest.h"

using jojo::perception::algorithm::DBSCAN;
using jojo::perception::algorithm::DBSCAN_velocity;
using jojo::perception::base::Point3DF;
using jojo::perception::base::PointF;

TEST(DbscanTest, ClustersSupportedPointTypes) {
  const std::vector<Point3DF> points = {
      {1.0, 1.0, 1.0}, {1.5, 1.5, 1.5}, {0.5, 0.5, 1.5}, {2.0, 2.0, 2.0},
      {2.0, 2.0, 3.0}, {8.0, 8.0, 1.0}, {8.5, 8.5, 2.0}, {9.0, 9.0, 8.0}};
  const std::vector<PointF> points_v = {
      {1.0, 1.0, 1.0, 1.0}, {1.5, 1.5, 1.5, 1.0},
      {0.5, 0.5, 1.5, 1.0}, {2.0, 2.0, 2.0, 1.0},
      {2.0, 2.0, 3.0, 1.0}, {8.0, 8.0, 1.0, 1.0},
      {8.5, 8.5, 2.0, 2.0}, {9.0, 9.0, 8.0, 1.0}};

  DBSCAN<Point3DF> dbscan_xyz;
  ASSERT_TRUE(dbscan_xyz.set_params(1.5, 2));
  dbscan_xyz.SetInputCloud(points);
  ASSERT_TRUE(dbscan_xyz.Run());

  DBSCAN<PointF> dbscan_xyzi;
  ASSERT_TRUE(dbscan_xyzi.set_params(1.5, 2));
  dbscan_xyzi.SetInputCloud(points_v);
  ASSERT_TRUE(dbscan_xyzi.Run());

  EXPECT_EQ(dbscan_xyz.GetLabels(), dbscan_xyzi.GetLabels());
}

TEST(DbscanTest, ExpandsThroughDensityConnectedCorePoints) {
  struct Point {
    double x;
    double y;
    double z;
  };

  // The endpoints are border points and the middle points are core points.
  // All points are density-connected and therefore belong to one cluster.
  const std::vector<Point> points = {
      {0.0, 0.0, 0.0}, {0.9, 0.0, 0.0}, {1.8, 0.0, 0.0},
      {2.7, 0.0, 0.0}, {3.6, 0.0, 0.0}};

  DBSCAN<Point> dbscan;
  ASSERT_TRUE(dbscan.set_params(1.0, 3));
  dbscan.SetInputCloud(points);
  ASSERT_TRUE(dbscan.Run());

  EXPECT_EQ(dbscan.GetLabels(), std::vector<int>({1, 1, 1, 1, 1}));
  std::vector<std::vector<Point>> clusters;
  ASSERT_TRUE(dbscan.OutputCluster(clusters));
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().size(), points.size());
}

TEST(DbscanTest, RegionQueryDoesNotDependOnClusterLabels) {
  struct Point {
    double x;
    double y;
    double z;
  };

  DBSCAN<Point> dbscan;
  ASSERT_TRUE(dbscan.set_params(1.0, 2));
  dbscan.SetInputCloud(
      {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {5.0, 0.0, 0.0}});
  ASSERT_TRUE(dbscan.Run());

  std::vector<int> neighbors;
  dbscan.RegionQuery(0, 1.0, neighbors);
  std::sort(neighbors.begin(), neighbors.end());
  EXPECT_EQ(neighbors, std::vector<int>({0, 1}));

  // Verify the compatibility overload has the same pure-query behavior.
  std::vector<int> labels = dbscan.GetLabels();
  neighbors.clear();
  dbscan.RegionQuery(0, 1.0, labels, neighbors);
  std::sort(neighbors.begin(), neighbors.end());
  EXPECT_EQ(neighbors, std::vector<int>({0, 1}));
}

TEST(DbscanTest, NoiseCanBeAbsorbedAsBorderPoint) {
  struct Point {
    double x;
    double y;
    double z;
  };

  // Point 0 is visited first and initially marked noise. Point 1 is a core
  // point, so point 0 must later be absorbed into its cluster as a border.
  DBSCAN<Point> dbscan;
  ASSERT_TRUE(dbscan.set_params(1.0, 3));
  dbscan.SetInputCloud(
      {{0.0, 0.0, 0.0}, {0.9, 0.0, 0.0}, {1.0, 0.1, 0.0}});
  ASSERT_TRUE(dbscan.Run());

  EXPECT_EQ(dbscan.GetLabels(), std::vector<int>({1, 1, 1}));
}


TEST(DbscanTest, VelocityExtensionOverridesOnlyDistanceMetric) {
  struct PointXYZV {
    double x;
    double y;
    double z;
    double v;
  };

  DBSCAN_velocity<PointXYZV> dbscan;
  ASSERT_TRUE(dbscan.set_params(0.5, 2));

  dbscan.SetInputCloud({{0.0, 0.0, 0.0, 0.0},
                        {0.1, 0.0, 0.0, 0.1},
                        {0.2, 0.0, 0.0, 10.0}});
  ASSERT_TRUE(dbscan.Run());

  // The first two points are close in both position and velocity. The third
  // point is spatially close but excluded by the overridden velocity metric.
  EXPECT_EQ(dbscan.GetLabels(), std::vector<int>({1, 1, -1}));
}
