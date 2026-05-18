#pragma once

#include <unistd.h>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include "eigen3/Eigen/Core"

#include "NvInfer.h"
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"
#include "cuda.h"
#include "cuda_device_runtime_api.h"
#include "cuda_runtime_api.h"
#include "cudnn.h"

// #define YOLOV5
#define YOLOV8

#if defined(YOLOV5)
#include "modules/perception/camera_detection_single_stage/detector/yolov5/yolov5_wrapper.h"
#elif defined(YOLOV8)
#include "modules/perception/camera_detection_single_stage/detector/yolov8/yolov8_wrapper.h"
#endif

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/box_extra.h"

// 工作空间粒度必须远大于模块粒度。
namespace jojo {
namespace perception {
namespace cdss {

// 自己写多线程推理，不靠谱，从DeepStream中获取相关代码
// SensorFusion =>> ImageDetector =>> LidarProj
#if defined(YOLOV5)
class YoloObstacleDetector : public Yolov5Wrapper {
#elif defined(YOLOV8)
class YoloObstacleDetector : public Yolov8Wrapper {
#endif
 public:
  YoloObstacleDetector();
  YoloObstacleDetector(const char* m_loadpath);
  virtual ~YoloObstacleDetector();

  void Init(const std::string& engine_file);
  bool isInited() const { return initialized_; };

  void Start();
  void Stop();

  void Run();

  // Infering
  void YOLO(cv::Mat& infer_img,
            std::vector<jojo::perception::base::Object>& detection,
            int input_W = kInputW, int input_H = kInputH, bool show = false);

  // Postprocessing
  // 转化成 语义类型，防止应为 class_id 不一致导致 目标类型 错误
  jojo::perception::base::ObjectType SwitchBoxTypeWrapper(const int& class_id);

  void DrawObjectRec(cv::Mat& image,
                     jojo::perception::base::ObjectType& classType,
                     cv::Rect& rec);

 protected:
  bool InitEngine(const std::string& engine_file);

  std::atomic_bool initialized_{false}, isRunning_{false};
};

#if 0
class YoloObstacleDetector : public BaseObstacleDetector {
 public:
  YoloObstacleDetector() = default;
  virtual ~YoloObstacleDetector() = default;

  bool Init(const ObstacleDetectorInitOptions &options) override;

  bool Detect(onboard::CameraFrame *frame) override;

  std::string Name() const override { return "SmokeObstacleDetector"; }

protected:
  void InitImageOffset(const smoke::ModelParam &model_param);
  void InitImageSize(const smoke::ModelParam &model_param);
  void InitParam(const smoke::ModelParam &model_param);
  void InitObstacleTypes();

private:
  smoke::ModelParam model_param_;
  ObstacleDetectorInitOptions options_;
  std::vector<base::ObjectSubType> types_;

  int width_ = 0;
  int height_ = 0;
  int offset_y_ = 0;

  int ori_cycle_ = 1;
  float confidence_threshold_ = 0.f;
  float border_ratio_ = 0.f;

  smoke::MinDims min_dims_;

  std::shared_ptr<base::Image8U> image_;
};
#endif

}  // namespace cdss
}  // namespace perception
}  // namespace jojo
