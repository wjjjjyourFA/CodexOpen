#include "modules/perception/camera_detection_single_stage/detector/yolo_obstacle_detector.h"

namespace jojo {
namespace perception {
namespace cdss {
namespace base = jojo::perception::base;

YoloObstacleDetector::YoloObstacleDetector() {
  // yolo_init();
}

YoloObstacleDetector::YoloObstacleDetector(const char* m_loadpath) {}

YoloObstacleDetector::~YoloObstacleDetector() {
  // delete tool_indeal;

  // Release stream and buffers

  // 当一个类派生自另一个类时，派生类的析构函数在销毁对象时
  // 会自动调用基类的析构函数。你不需要手动调用基类的析构函数。
  // ~Yolov5Wrapper();
  // ~Yolov8Wrapper();
}

void YoloObstacleDetector::Init(const std::string& engine_file) {
  if (initialized_) return;

  if (!this->InitEngine(engine_file)) {
    initialized_ = false;
    return;
  }

  initialized_ = true;
}

bool YoloObstacleDetector::InitEngine(const std::string& engine_file) {
  if (initialized_) return true;

  // yolov5
  if (!yolo_init(engine_file)) {
    printf("... loading engine failed\n");
    return false;
  }
  printf("... loading engine succeeded\n");

  return true;
}

void YoloObstacleDetector::Start() { isRunning_ = true; }

void YoloObstacleDetector::Stop() {
  isRunning_   = false;
  initialized_ = false;
}

void YoloObstacleDetector::Run() {
  while (isRunning_) {
    usleep(10000000);
  }
}

void YoloObstacleDetector::YOLO(cv::Mat& infer_img,
                                std::vector<base::Object>& detection,
                                int input_W, int input_H, bool show) {
  // 浅拷贝 传参数 修改原图
  cv::Mat img = infer_img;
  if (img.empty()) return;

  // 这里的 detection 结构体，来自于各检测器内部的定义，这里转换为通用结构体 base::Object
  std::vector<std::vector<Detection>> det_result;

  // !! 复用的 原 yolo 代码，用于 batchsize，这里强制使用的单帧；
#if defined(YOLOV5)
  det_result = yolov5(img);
#elif defined(YOLOV8)
  det_result = yolov8(img);
#endif

  // 20251224 only one frame
  detection.clear();
  size_t total = 0;
  for (const auto& res : det_result) {
    total += res.size();
  }
  detection.resize(total);

  size_t idx = 0;
  for (auto& res : det_result) {
    for (auto& obj : res) {
      cv::Rect r = get_rect(img, obj.bbox);

      // result => up_left(x1,y1) down_right(x2,y2)
      auto& result = detection[idx++];
      // don't use this id for class_id; 模型检测 id 映射到 定义的 id
      // result.id = obj.class_id;
      result.type = jojo::perception::base::SwitchBoxTypeWrapper(obj.class_id);

      result.confidence = obj.conf;

      result.camera_supplement.box =
          jojo::perception::base::tlwh2box(r.x, r.y, r.width, r.height);
      // YOLOV5 可以省略这个限制
      // x1 = ClampValue(x1, 0, img.cols - 1);
      // y1 = ClampValue(y1, 0, img.rows - 1);
      // x2 = ClampValue(x2, 0, img.cols - 1);
      // y2 = ClampValue(y2, 0, img.rows - 1);

      // detection.push_back(result);
      // detection.emplace_back(std::move(result));

      // obj.bbox => center_x center_y width height
      // r => up_left(x,y), width height
      if (show) {
        this->DrawObjectRec(img, result.type, r);
      }
    }
  }

  if (show) {
    cv::namedWindow("yolo det result", cv::WINDOW_NORMAL);
    cv::imshow("yolo det result", img);
    cv::waitKey(1);
  }
}

void YoloObstacleDetector::DrawObjectRec(cv::Mat& image,
                                         base::ObjectType& classType,
                                         cv::Rect& rec) {
  // 使用 BoxTypetoColor 获取颜色
  cv::Scalar color = base::BoxTypetoColor(classType);

  // 使用获取的颜色绘制矩形
  cv::rectangle(image, rec, color, 5);

  std::string name = base::BoxTypetoString(classType);

  cv::putText(image, name, cv::Point(rec.x, rec.y - 1), cv::FONT_HERSHEY_PLAIN,
              1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  // cv::putText(image, name, cv::Point(rec.x, rec.y - 1),
  //             cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 255, 0), 2);
}

}  // namespace cdss
}  // namespace perception
}  // namespace jojo
