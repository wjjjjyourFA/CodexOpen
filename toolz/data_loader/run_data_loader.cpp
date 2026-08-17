#include <chrono>

#include <opencv2/opencv.hpp>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "toolz/data_loader/group_convert.h"

using namespace jojo::tools;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "DataLoaderDataSet";
  std::string config_path = "./../../config/DataLoader/DataLoaderDataSet.ini";
  // clang-format on

  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<jojo::tools::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  std::string if_config_path = "./../../config/DataLoader/InterfaceDataSet.ini";

  auto interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
  interface_config->set_name(name);
  interface_config->LoadConfig(if_config_path);

  auto group_convert = std::make_shared<GroupConvertDataSet>();
  if (!group_convert->Init(runtime_config, interface_config)) {
    std::cerr << "[ERROR] Failed to initialize GroupConvert" << std::endl;
    return 1;
  }

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("Lidar Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);

  // 计时器
  auto start_time = std::chrono::steady_clock::now();
  std::cout << std::fixed << std::setprecision(0);

  std::cout << "Start to load data..." << std::endl;

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroupDataSet> group;
  while (!group_convert->IsEnd()) {
    auto base = group_convert->ReadNext();
    if (!base) {
      break;
    }
    group = std::static_pointer_cast<const MeasureGroupDataSet>(base);
    // std::cout << "Frame " << frame_idx
    //           << ": Image Time = " << group->camera.at(0).time
    //           << ", Lidar Time = " << group->lidar.time
    //           << ", Imu Time = " << group->imu.time
    //           << ", GNSS Time = " << group->gnss.time
    //           << std::endl;

    // 显示图像
    if (!group->camera.empty() && !group->camera.front().data.empty()) {
      // cv::Mat img_show = group->camera.at(0).data.clone();
      cv::Mat img_show = group->camera.front().data;
      cv::putText(img_show, "Frame: " + std::to_string(frame_idx),
                  cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 255, 0), 2);
      cv::imshow("Image", img_show);
      // cv::waitKey(0);
    }

    /* 显示 pcl::PointXYZI 点云
    if (viewer->contains("cloud")) {
      viewer->updatePointCloud(group->lidar.data, "cloud");
    } else {
      viewer->addPointCloud(group->lidar.data, "cloud");
    }
    */

    pcl::PointCloud<pcl::PointXYZI>::Ptr t(new pcl::PointCloud<pcl::PointXYZI>);
    const auto& src = group->lidar.data;
    /* way 1
    t->points.resize(src->points.size());
    for (size_t i = 0; i < src->points.size(); ++i) {
      const auto& pt = src->points[i];

      auto& p = t->points[i];
      // 逐字段赋值
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      // 如果原始点没有 intensity，可以自己赋值
      p.intensity = pt.intensity;
    }
    t->width    = t->points.size();
    t->height   = 1;
    t->is_dense = true;
    */
    // way 2
    pcl::copyPointCloud(*src, *t);

    viewer->removeAllPointClouds();
    // 按z轴着色渲染
    // clang-format off
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(group->lidar.data, "z");
    // viewer->addPointCloud(group->lidar.data, color_handler, "cloud");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(t, "z");
    viewer->addPointCloud(t, color_handler, "cloud");
    // clang-format on

    viewer->spinOnce(1);

    // 每10帧统计一下读取速度
    if (frame_idx % 10 == 0 && frame_idx > 0) {
      auto now = std::chrono::steady_clock::now();
      double elapsed_sec =
          std::chrono::duration_cast<std::chrono::duration<double>>(now -
                                                                    start_time)
              .count();
      double fps = frame_idx / elapsed_sec;
      std::cout << "[INFO] Processed " << frame_idx << " frames in "
                << elapsed_sec << " seconds, FPS = " << fps << std::endl;
    }

    // 等待按键
    char key = (char)cv::waitKey(1);  // 1ms，不阻塞主程序
    if (key == 'q' || key == 'Q') {
      break;
    }

    frame_idx++;
  }

  viewer->close();
  std::cout << "All frames processed. Total: " << frame_idx << " frames."
            << std::endl;
  return 0;
}
