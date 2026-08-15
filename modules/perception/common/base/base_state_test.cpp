#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/base/object.h"

int main() {
  jojo::perception::base::Object object;
  if (!object.polygon || !object.lidar_supplement.cloud ||
      !object.lidar_supplement.cloud_world) {
    return 1;
  }
  object.polygon->emplace_back(1.0f, 2.0f, 3.0f);
  object.lidar_supplement.cloud->emplace_back();
  object.lidar_supplement.point_ids.push_back(42);
  object.camera_supplement.sensor_name = "camera";
  object.drop_num = 2;
  object.b_cipv = true;
  object.Reset();
  if (!object.polygon->empty() || !object.lidar_supplement.cloud->empty() ||
      !object.lidar_supplement.point_ids.empty() ||
      !object.camera_supplement.sensor_name.empty() || object.drop_num != 0 ||
      object.b_cipv) {
    return 2;
  }

  apollo::perception::base::Frame frame;
  frame.lidar_frame_supplement.on_use = true;
  frame.lidar_frame_supplement.cloud_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  frame.Reset();
  if (frame.lidar_frame_supplement.on_use ||
      frame.lidar_frame_supplement.cloud_ptr) {
    return 3;
  }
  return 0;
}
