#include <chrono>

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "tools/data_loader/group_convert.h"

using namespace jojo::tools;

int main(int argc, char** argv) {
  std::string name        = "DataLoader";
  std::string config_path = "./../../config/DataLoader/DataLoader.ini";

  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<RuntimeConfigOffline>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  auto data_loader = std::make_shared<GroupConvert>();
  data_loader->Init(runtime_config);

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("Lidar Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);

  // 计时器
  auto start_time = std::chrono::steady_clock::now();
  std::cout << std::fixed << std::setprecision(0);

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroup> group;
  while (!data_loader->IsEnd()) {
    group = data_loader->ReadNext();
    std::cout << "Frame " << frame_idx
              << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Imu Time = " << group->imu.time << std::endl;

    // 显示图像
    if (!group->camera.at(0).data.empty()) {
      cv::Mat img_show = group->camera.at(0).data.clone();
      cv::putText(img_show, "Frame: " + std::to_string(frame_idx),
                  cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 255, 0), 2);
      cv::imshow("Image", img_show);
      // cv::waitKey(0);
    }

    /*  // 显示点云
    if (viewer->contains("cloud")) {
      viewer->updatePointCloud(group->lidar.data, "cloud");
    } else {
      viewer->addPointCloud(group->lidar.data, "cloud");
    }
    */

    viewer->removeAllPointClouds();
    // 按z轴着色渲染
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
        color_handler(group->lidar.data, "z");
    viewer->addPointCloud(group->lidar.data, color_handler, "cloud");

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
