#include <deepsort_wrapper.h>

using namespace std;

DeepSortWrapper::DeepSortWrapper() {}

DeepSortWrapper::~DeepSortWrapper() {}

void DeepSortWrapper::set_wts(std::string in_wts_name) {
  wts_name = in_wts_name;
}

bool DeepSortWrapper::sort_init(std::string in_engine_name) {
  if (in_engine_name.empty()) {
    return false;
  }

  this->deep_sort = std::make_shared<DeepSort>(
      in_engine_name, batchSize, featureDim, gpuID, &gLogger_deepsort);
  this->deep_sort->init();

  return true;
}

bool DeepSortWrapper::sort_init(std::string in_engine_name,
                                float _maxCosineDist, int _maxBudget,
                                float _maxIouDistance, int _maxAge,
                                int _nInit) {
  if (in_engine_name.empty()) {
    return false;
  }

  this->deep_sort = std::make_shared<DeepSort>(
      in_engine_name, batchSize, featureDim, gpuID, &gLogger_deepsort);
  this->deep_sort->init(_maxCosineDist, _maxBudget, _maxIouDistance, _maxAge,
                        _nInit);

  return true;
}

std::vector<jojo::perception::base::Object> DeepSortWrapper::deepsort(
    cv::Mat& input_image, std::vector<jojo::perception::base::Object>& obj,
    bool show) {
  // image input here !
  // result base on this image !
  if (input_image.empty()) {
    std::cout << "image is empty" << std::endl;
    return {};
  };
  cv::Mat img1 = input_image;
  cv::Mat img;
  // cv::resize(img1, img, cv::Size(kInputW, kInputH));
  img = img1;
  // TODO：深拷贝（避免多线程问题）
  // img = img1.clone();

  std::vector<DetectBox> box;
  this->ConvertDetectionsToInternalBoxes(obj, box);

  this->deep_sort->sort(img, box);

  std::vector<jojo::perception::base::Object> res;
  this->ConvertInternalBoxesToTrackResults(box, res);

  // /*
  // Draw bounding boxes
  if (show) {
    // this->draw_bbox(img, obj);
    this->draw_bbox(img, res);
  }
  // */

  return res;
}

// from yolov8/src/postprocess.cpp
void DeepSortWrapper::draw_bbox(
    cv::Mat& img, std::vector<jojo::perception::base::Object>& res) {
  for (size_t j = 0; j < res.size(); j++) {
    auto& obj    = res[j].camera_supplement.box;
    float width  = obj.xmax - obj.xmin;
    float height = obj.ymax - obj.ymin;

    if (height <= 1e-6) continue;
    bool vertical = (width / height) > 1.6;

    if (obj.Area() > 20 && !vertical) {
      cv::Point lt(obj.xmin, obj.ymin);
      cv::Point rb(obj.xmax, obj.ymax);
      cv::rectangle(img, lt, rb, cv::Scalar(0x27, 0xC1, 0x36), 2);
      std::string text =
          cv::format("ID:%d", (int)res[j].camera_supplement.local_track_id);
      cv::putText(img, text, lt, cv::FONT_HERSHEY_PLAIN, 1.2,
                  cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
  }
}

void DeepSortWrapper::ConvertDetectionsToInternalBoxes(
    const std::vector<jojo::perception::base::Object>& det,
    std::vector<DetectBox>& in_boxes) {
  in_boxes.clear();

  in_boxes.resize(det.size());
  for (size_t i = 0; i < det.size(); ++i) {
    const auto& d = det[i];
    const auto& b = d.camera_supplement.box;

    auto& box = in_boxes[i];
    box.x1    = b.xmin;
    box.y1    = b.ymin;
    box.x2    = b.xmax;
    box.y2    = b.ymax;

    box.trackID = -1;  // 未跟踪状态

    box.confidence = d.confidence;
    box.classID    = jojo::perception::base::BoxTypeToInt(d.type);
  }
}

void DeepSortWrapper::ConvertInternalBoxesToTrackResults(
    const std::vector<DetectBox>& in_boxes,
    std::vector<jojo::perception::base::Object>& track_results) {
  track_results.clear();

  track_results.resize(in_boxes.size());
  for (size_t i = 0; i < in_boxes.size(); ++i) {
    const auto& b = in_boxes[i];

    auto& res = track_results[i];
    res.type  = jojo::perception::base::SwitchBoxTypeWrapper(b.classID);

    res.confidence = b.confidence;

    res.camera_supplement.local_track_id = b.trackID;

    auto& box = res.camera_supplement.box;
    box.xmin  = b.x1;
    box.ymin  = b.y1;
    box.xmax  = b.x2;
    box.ymax  = b.y2;
  }
}