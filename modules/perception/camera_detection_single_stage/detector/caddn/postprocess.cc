/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera_detection_single_stage/detector/caddn/postprocess.h"

#include "cyber/common/log.h"
#include "modules/perception/common/base/box.h"

namespace apollo {
namespace perception {
namespace camera {

/* 
  将lidar坐标系下的bbox转换到camera坐标系下
  前左上 ==> 右下前
*/
void Bbox3dLidar2Camera(const Eigen::Matrix<float, 3, 4> &V2C,
                        const Eigen::Matrix<float, 3, 3> &R,
                        const float *bbox_lidar,
                        std::vector<float> *bbox_camera) {
  float x = *(bbox_lidar + 0);
  float y = *(bbox_lidar + 1);
  float z = *(bbox_lidar + 2);
  float l = *(bbox_lidar + 3);
  float w = *(bbox_lidar + 4);
  float h = *(bbox_lidar + 5);
  float r = *(bbox_lidar + 6);
  // 旋转方向取反 + 前向轴差 90° 的补偿
  r = -r - M_PI / 2.0;
  // 4*1
  Eigen::Vector4f xyz_lidar;
  xyz_lidar << x, y, z - h / 2.0, 1.0;
  Eigen::Matrix<float, 1, 3> pts_rect =
      xyz_lidar.transpose() * (V2C.transpose() * R.transpose());
  std::vector<float> final_result{
      pts_rect(0), pts_rect(1), pts_rect(2), l, h, w, r};
  bbox_camera->assign(final_result.data(),
                      final_result.data() + final_result.size());
}

// /*
void FillCaddnBbox3d(jojo::perception::base::ObjectPtr obj, const float *bbox) {
  // x y c l w h
  obj->size[0] = bbox[3];
  obj->size[1] = bbox[4];
  obj->size[2] = bbox[5];

  obj->confidence = bbox[6];

  // 相机坐标系
  obj->camera_supplement.local_center[0] = bbox[0];
  obj->camera_supplement.local_center[1] = bbox[1];
  obj->camera_supplement.local_center[2] = bbox[2];
}
// */

}  // namespace camera
}  // namespace perception
}  // namespace apollo
