#include "modules/perception/tools/pcl/pcl_viewer.h"

void SpinViewer(pcl::visualization::PCLVisualizer::Ptr viewer) {
  while (!viewer->wasStopped()) {
    viewer->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}