#include <algorithm>
#include <vector>

#include "modules/perception/common/algorithm/image_processing/hough_transfer.h"
#include "modules/perception/common/algorithm/point_cloud_processing/dbscan.h"

namespace {

struct Point {
  double x;
  double y;
  double z;
};

}  // namespace

int main() {
  apollo::perception::algorithm::HoughTransfer hough;
  if (hough.Init(0, 4, 1.0f, 1.0f)) return 1;
  if (!hough.Init(4, 4, 1.0f, 1.0f)) return 2;
  const std::vector<int> image = {0, 0, 0, 1, 0, 0, 1, 0,
                                  0, 1, 0, 0, 1, 0, 0, 0};
  if (!hough.ImageVote(image, true)) return 3;
  std::vector<apollo::perception::algorithm::HoughLine> lines;
  if (!hough.GetLines(4, 2, 4, true, &lines) || lines.empty()) return 4;
  const auto votes_before = hough.get_vote_map();
  hough.PointVote(-1, -1, true);
  if (votes_before != hough.get_vote_map()) return 5;

  jojo::perception::algorithm::DBSCAN<Point> dbscan;
  if (dbscan.set_params(0.0, 2)) return 6;
  if (!dbscan.set_params(0.5, 2)) return 7;
  dbscan.SetInputCloud({{0.0, 0.0, 0.0}, {0.1, 0.1, 0.0},
                        {5.0, 5.0, 0.0}, {5.1, 5.1, 0.0},
                        {20.0, 20.0, 20.0}});
  if (!dbscan.Run()) return 8;
  std::vector<std::vector<Point>> clusters;
  if (!dbscan.OutputCluster(clusters) || clusters.size() != 2) return 9;
  std::vector<size_t> sizes = {clusters[0].size(), clusters[1].size()};
  std::sort(sizes.begin(), sizes.end());
  if (sizes != std::vector<size_t>({2, 2})) return 10;
  
  return 0;
}
