#include "modules/perception/tools/common/show_data_3d.h"

void show3d_lidar_data(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                       const std::string& viewer_name) {
  // 适用于 单线程、局部使用。
  auto viewer =
      boost::make_shared<pcl::visualization::PCLVisualizer>(viewer_name);
  // viewer.setBackgroundColor(0.0, 0.0, 0.0);  // 设置背景色
  // viewer.removeAllPointClouds();

  // clang-format off
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  if (!viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgb)) {
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
  }
  // clang-format on

  // viewer.spin();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
}

/* VTK
void show3d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       const std::string &viewer_name) {
  pcl::visualization::PCLVisualizer viewer(viewer_name);

  // 根据强度字段 "intensity" 为点云着色
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
      intensity_color(cloud, "intensity");

  if (intensity_color.isCapable()) {
    viewer.addPointCloud<pcl::PointXYZI>(cloud, intensity_color, "cloud_xyzi");
  } else {
    viewer.addPointCloud<pcl::PointXYZI>(cloud, "cloud_xyzi");
  }

  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_xyzi");

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }
}
*/
void show3d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                       const std::string& viewer_name) {
  pcl::visualization::CloudViewer viewer(viewer_name);

  viewer.showCloud(cloud);

  while (!viewer.wasStopped()) {
    // 不需要 spinOnce
  }
}

// 使用 shared_ptr 管理 PCLVisualizer 以支持多线程共享
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
//   new pcl::visualization::PCLVisualizer(name));
void show3d_lidar_data_shared(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& cloud_name) {
  // clang-format off
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  if (!viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name)) {
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name);
  }
  // clang-format on

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // 暂停一段时间以避免CPU占用过高
    boost::this_thread::sleep(boost::posix_time::microseconds(1));
  }
}

void show3d_lidar_data_realtime(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
    const std::string& cloud_name) {
  // clang-format off
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  if (!viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name)) {
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name);
  }
  // clang-format on

  viewer->spinOnce(10);  // 每 10 ms 刷新一次窗口
}

void show3d_box3d_shared(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
    const base::Point3DF (&vertex_p)[8],  // 使用数组传递 8 个 Point3DF 对象
    const std::string& box_name) {
  // PointCloudColorHandler 类，用于指定如何为点云中的每个点着色
  // "y" 是字段的名称，表示我们将根据点云中每个点的 y 坐标值来给点云着色。
  if (cloud_ptr) {  // 检查 cloud_ptr 是否为空
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(
        cloud_ptr, "y");
    viewer->addPointCloud(cloud_ptr, rgb, box_name);
  }
  //            7 -------- 4
  //           /|         /|
  //          6 -------- 5 .
  //          | |     k3 | |
  //          . 3 -------- 0
  //        k2|/         |/k0
  //          2 -------- 1
  //               k1

  pcl::PointXYZ vertex[8];
  // 填充顶点数据
  for (int i = 0; i < 8; i++) {
    vertex[i].x = vertex_p[i].x;
    vertex[i].y = vertex_p[i].y;
    vertex[i].z = vertex_p[i].z;
  }

  // 连接的顶点对
  int connections[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},  // 底面
      {1, 5}, {2, 6}, {3, 7}, {0, 4},  // 侧面
      {4, 5}, {5, 6}, {6, 7}, {4, 7}  // 顶面
  };

  // 绘制所有线条
  for (int i = 0; i < 12; i++) {
    int start = connections[i][0];
    int end   = connections[i][1];

    // 为每条线设置不同的ID，并连接相应的顶点
    std::string line_id = "line_" + std::to_string(i + 1);
    viewer->addLine<pcl::PointXYZ>(vertex[start], vertex[end], 1.0f, 0.0f, 0.0f,
                                   line_id);
  }

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, box_name);
  viewer->addCoordinateSystem(1.0);

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  };
}
