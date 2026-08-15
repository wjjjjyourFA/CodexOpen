#include "modules/perception/tools/pcl/pcl_viewer.h"

void SpinViewer(pcl::visualization::PCLVisualizer::Ptr viewer) {
  if (!viewer) {
    return;
  }

  jojo::perception::tools::ViewerRunner runner(viewer);

  runner.Run();
}
