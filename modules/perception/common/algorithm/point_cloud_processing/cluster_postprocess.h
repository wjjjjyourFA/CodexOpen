#ifndef __CLUSTER_POSTPROCESS_H
#define __CLUSTER_POSTPROCESS_H

#include <pcl/common/common.h>

#include "modules/perception/common/base/box_extra.h"
#include "modules/perception/common/base/box3d_extra.h"
#include "modules/perception/common/base/segment.h"

namespace jojo {
namespace perception {
namespace algorithm {

// 只遍历一次 + PCL内部优化
void CalculateSegAttributeLegacy(
    std::shared_ptr<jojo::perception::base::Segment>& seg);

// 纯手写遍历 额外 计算质心
void CalculateSegAttribute(
    std::shared_ptr<jojo::perception::base::Segment>& seg);

void GetLargestCluster(
    std::vector<std::shared_ptr<jojo::perception::base::Segment>>& SegVector,
    std::shared_ptr<jojo::perception::base::Segment>& result);

void GetClosestAmongTop2Clusters(
    std::vector<std::shared_ptr<jojo::perception::base::Segment>>& SegVector,
    std::shared_ptr<jojo::perception::base::Segment>& result);

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo

#endif