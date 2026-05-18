#include "modules/perception/camera_tracking/camera_tracking.h"

namespace jojo {
namespace perception {
namespace ct {
namespace base = jojo::perception::base;
namespace cdss = jojo::perception::cdss;

CameraTracking::CameraTracking() {
  image_detector = std::make_shared<cdss::YoloObstacleDetector>();
}

CameraTracking::~CameraTracking() {}

void CameraTracking::Init(const std::string& det_engine_file,
                          const std::string& sort_engine_file) {
  if (initialized_) return;

  if (!this->InitEngine(det_engine_file, sort_engine_file)) {
    initialized_ = false;
    return;
  }

  initialized_ = true;
}

bool CameraTracking::InitEngine(const std::string& det_engine_file,
                                const std::string& sort_engine_file) {
  if (initialized_) return true;

  image_detector->Init(det_engine_file);
  if (!image_detector->isInited()) {
    return false;
  }

#if defined(DEEPSORT)
  if (sort_engine_file.empty()) {
    return false;
  }
  if (!sort_init(sort_engine_file, hps_.maxCosineDist, hps_.maxBudget,
                 hps_.maxIouDistance, hps_.maxAge, hps_.nInit)) {
    printf("... loading engine failed\n");
    return false;
  }
  printf("... loading engine succeeded\n");
#elif defined(BYTETRACK)
  if (!sort_init(hps_.frame_rate, hps_.track_buffer)) {
    return false;
  }
#endif

  return true;
}

void CameraTracking::Start() {
  isRunning_ = true;

  this->image_detector->Start();
}

void CameraTracking::Stop() {
  isRunning_   = false;
  initialized_ = false;

  this->image_detector->Stop();
}

void CameraTracking::Run() {
  while (isRunning_) {
    usleep(10000000);
  }
}

void CameraTracking::DetectionAndTracking(
    cv::Mat& infer_img, std::vector<jojo::perception::base::Object>& track,
    int input_W, int input_H, bool show) {
  cv::Mat img = infer_img;
  if (img.empty()) return;

  // step 1: detection
  std::vector<jojo::perception::base::Object> detections;
  image_detector->YOLO(img, detections, img.cols, img.rows, false);

  // step 2: sort tracking
  std::vector<jojo::perception::base::Object> track_result;

#if defined(DEEPSORT)
  track_result = deepsort(img, detections, false);
#elif defined(BYTETRACK)
  track_result = bytetrack(img, detections, false);
#endif

  // 20251224 only one frame
  track.clear();
  track = track_result;

  if (show) {
    for (auto& res : track_result) {
      auto& track_id = res.camera_supplement.local_track_id;

      auto& box = res.camera_supplement.box;

      int x = static_cast<int>(box.xmin);
      int y = static_cast<int>(box.ymin);
      int w = static_cast<int>(box.xmax - box.xmin);
      int h = static_cast<int>(box.ymax - box.ymin);

      // 防止非法框
      if (w <= 0 || h <= 0) {
        continue;
      }

      cv::Rect rect(x, y, w, h);
      this->DrawObjectRec(img, res.type, rect, track_id);
    }

    cv::namedWindow("det track result", cv::WINDOW_NORMAL);
    cv::imshow("det track result", img);
    cv::waitKey(1);
  }
}

void CameraTracking::DrawObjectRec(
    cv::Mat& image, jojo::perception::base::ObjectType& classType,
    cv::Rect& rec, int& trackID) {
  // 1. 复用 detector 的绘制（框 + 类别）
  this->image_detector->DrawObjectRec(image, classType, rec);

  // 2. 追加绘制 trackID
  std::string id_text = "ID:" + std::to_string(trackID);

  // 放在类别文字下方，避免重叠
  cv::Point id_pos(rec.x + 5, rec.y + 15);

  cv::putText(image, id_text, id_pos, cv::FONT_HERSHEY_PLAIN, 1.2,
              cv::Scalar(255, 255, 255), 2);
}

void CameraTracking::get_track_box() {
  //   vector<DetectBox> res = global_res;
  //   float iou             = 0;
  //   int index             = 0;
  //   for (int i = 0; i < res.size(); i++) {
  //     float det_box[4];
  //     det_box[0]    = res[i].x1;
  //     det_box[1]    = res[i].y1;
  //     det_box[2]    = res[i].x2;
  //     det_box[3]    = res[i].y2;
  //     float iou_cur = getiou(det_box, obj);
  //     if (iou_cur > iou) {
  //       index = i;
  //       iou   = iou_cur;
  //     }
  //   }
  //   if (index >= 0 && index < res.size() && iou != 0) {
  //     trackID = res[index].trackID;
  //   }
  //   cout << "get trackID: " << trackID << endl;
}

// from yolov8/src/postprocess.cpp
float CameraTracking::iou(float lbox[4], float rbox[4]) {
  float interBox[] = {
      (std::max)(lbox[0], rbox[0]),
      (std::min)(lbox[2], rbox[2]),
      (std::max)(lbox[1], rbox[1]),
      (std::min)(lbox[3], rbox[3]),
  };

  if (interBox[2] > interBox[3] || interBox[0] > interBox[1]) return 0.0f;

  float interBoxS = (interBox[1] - interBox[0]) * (interBox[3] - interBox[2]);
  float unionBoxS = (lbox[2] - lbox[0]) * (lbox[3] - lbox[1]) +
                    (rbox[2] - rbox[0]) * (rbox[3] - rbox[1]) - interBoxS;
  return interBoxS / unionBoxS;
}

}  // namespace ct
}  // namespace perception
}  // namespace jojo
