#include "modules/perception/common/lidar/convert/velodyne.h"

int main() {
  pcl::PointCloud<velodyne_ros::PointXYZIRT>::Ptr empty;
  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZIRT>>();

  if (VdToPcl(empty, output)) return 1;

  auto input = std::make_shared<pcl::PointCloud<velodyne_ros::PointXYZIRT>>();
  velodyne_ros::PointXYZIRT point;
  point.x         = 1.0f;
  point.y         = 2.0f;
  point.z         = 3.0f;
  point.intensity = 4.0f;
  point.ring      = 5;
  point.time      = 6.0f;
  input->push_back(point);
  input->width  = 1;
  input->height = 1;

  if (!VdToPcl(input, output) || output->size() != 1) return 2;
  if ((*output)[0].ring != 5 || (*output)[0].timestamp != 6.0) return 3;

  return 0;
}
