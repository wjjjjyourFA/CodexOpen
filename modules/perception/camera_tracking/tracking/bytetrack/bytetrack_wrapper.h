#ifndef ByteTrackWrapper_H
#define ByteTrackWrapper_H

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "include/BYTETracker.h"
#include "include/dataType.h"
// #include "include/logging.h"

#include "modules/perception/common/base/object.h"

// static Logger gLogger_bytetrack;
// using namespace nvinfer1;
// namespace base = jojo::perception::base;

class ByteTrackWrapper {
 public:
  ByteTrackWrapper();
  virtual ~ByteTrackWrapper();

 public:
  bool sort_init(int frame_rate, int track_buffer);
  bool sort_init(int frame_rate, int track_buffer, float track_thresh,
                 float high_thresh, float match_thresh);

  std::vector<jojo::perception::base::Object> bytetrack(
      cv::Mat& input_image, std::vector<jojo::perception::base::Object>& obj,
      bool show = false);

  void ConvertDetectionsToInternalBoxes(
      const std::vector<jojo::perception::base::Object>& det,
      std::vector<bytetracker::Object>& in_boxes);

  void ConvertInternalBoxesToTrackResults(
      const std::vector<bytetracker::STrack>& in_boxes,
      std::vector<jojo::perception::base::Object>& track_results);

 protected:
  int kBatchSize = 1;  // 推理图像数量

  void draw_bbox(cv::Mat& img,
                 std::vector<jojo::perception::base::Object>& res);

 private:
  std::shared_ptr<bytetracker::BYTETracker> byte_tracker;
};

#endif  // ByteTrackWrapper_H
