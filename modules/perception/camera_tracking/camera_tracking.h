#ifndef CAMERA_TRACKING_H
#define CAMERA_TRACKING_H

#include "modules/perception/camera_detection_single_stage/detector/yolo_obstacle_detector.h"
#include "modules/perception/common/base/object.h"

// #define DEEPSORT
#define BYTETRACK

#if defined(DEEPSORT)
#include "modules/perception/camera_tracking/tracking/deepsort/deepsort_wrapper.h"
#elif defined(BYTETRACK)
#include "modules/perception/camera_tracking/tracking/bytetrack/bytetrack_wrapper.h"
#endif

namespace jojo {
namespace perception {
namespace ct {

struct CameraTrackingHyperparams {
  // DeepSort tracker
  float maxCosineDist  = 0.15;
  int maxBudget        = 200;
  float maxIouDistance = 0.5;
  int maxAge           = 70;
  int nInit            = 5;

  // ByteTrack
  int frame_rate     = 30;
  int track_buffer   = 30;
  float track_thresh = 0.5;
  float high_thresh  = 0.6;
  float match_thresh = 0.8;
};

#if defined(DEEPSORT)
class CameraTracking : public DeepSortWrapper {
#elif defined(BYTETRACK)
class CameraTracking : public ByteTrackWrapper {
#endif
 public:
  CameraTracking();
  virtual ~CameraTracking();

  void Init(const std::string& det_engine_file,
            const std::string& sort_engine_file = "");
  bool isInited() const { return initialized_; };

  void Start();
  void Stop();

  void Run();

  // Infering
  void DetectionAndTracking(cv::Mat& infer_img,
                            std::vector<jojo::perception::base::Object>& track,
                            int input_W = kInputW, int input_H = kInputH,
                            bool show = false);

  void DrawObjectRec(cv::Mat& image,
                     jojo::perception::base::ObjectType& classType,
                     cv::Rect& rec, int& trackID);

  void get_track_box();

  float iou(float lbox[4], float rbox[4]);

 protected:
  std::shared_ptr<jojo::perception::cdss::YoloObstacleDetector> image_detector;

  bool InitEngine(const std::string& engine_file,
                  const std::string& sort_engine_file = "");

  std::atomic_bool initialized_{false}, isRunning_{false};

 private:
  CameraTrackingHyperparams hps_;
};

}  // namespace ct
}  // namespace perception
}  // namespace jojo

#endif
