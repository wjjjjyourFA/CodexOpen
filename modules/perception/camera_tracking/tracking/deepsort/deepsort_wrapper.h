#ifndef DeepSortWrapper_H
#define DeepSortWrapper_H

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "include/datatype.h"
#include "include/deepsort.h"
#include "include/logging.h"

#include "modules/perception/common/base/object.h"

static Logger gLogger_deepsort;
using namespace nvinfer1;
namespace base = jojo::perception::base;

class DeepSortWrapper {
 public:
  DeepSortWrapper();
  virtual ~DeepSortWrapper();

 public:
  void set_wts(std::string in_wts_name);
  bool sort_init(std::string in_engine_name = "");
  bool sort_init(std::string in_engine_name, float _maxCosineDist,
                 int _maxBudget, float _maxIouDistance, int _maxAge,
                 int _nInit);
  std::vector<jojo::perception::base::Object> deepsort(
      cv::Mat& input_image, std::vector<jojo::perception::base::Object>& obj,
      bool show = false);

  void ConvertDetectionsToInternalBoxes(
      const std::vector<jojo::perception::base::Object>& det,
      std::vector<DetectBox>& in_boxes);

  void ConvertInternalBoxesToTrackResults(
      const std::vector<DetectBox>& in_boxes,
      std::vector<jojo::perception::base::Object>& track_results);

 protected:
  std::string wts_name    = "";
  std::string engine_name = "";

  int kBatchSize = 1;  // 推理图像数量

  void draw_bbox(cv::Mat& img,
                 std::vector<jojo::perception::base::Object>& res);

 private:
  std::shared_ptr<DeepSort> deep_sort;

  int batchSize  = 128;  // ReID 网络
  int featureDim = kFeatureDim;  // 256
  int gpuID      = 0;  // kGpuId
};

#endif  // DeepSortWrapper_H
