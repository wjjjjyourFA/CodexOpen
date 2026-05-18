#include <bytetrack_wrapper.h>

using namespace std;

ByteTrackWrapper::ByteTrackWrapper() {}

ByteTrackWrapper::~ByteTrackWrapper() {}

bool ByteTrackWrapper::sort_init(int frame_rate, int track_buffer) {
  if (frame_rate <= 0) {
    return false;
  }

  this->byte_tracker =
      std::make_shared<bytetracker::BYTETracker>(frame_rate, track_buffer);

  return true;
}

bool ByteTrackWrapper::sort_init(int frame_rate, int track_buffer,
                                 float track_thresh, float high_thresh,
                                 float match_thresh) {
  if (frame_rate <= 0) {
    return false;
  }

  this->byte_tracker = std::make_shared<bytetracker::BYTETracker>(
      frame_rate, track_buffer, track_thresh, high_thresh, match_thresh);

  return true;
}

std::vector<jojo::perception::base::Object> ByteTrackWrapper::bytetrack(
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

  // std::vector<STrack> stracks = byte_tracker->update(box);
  std::vector<bytetracker::Object> box;
  this->ConvertDetectionsToInternalBoxes(obj, box);

  std::vector<bytetracker::STrack> stracks;
  stracks = byte_tracker->update(box);

  std::vector<jojo::perception::base::Object> res;
  this->ConvertInternalBoxesToTrackResults(stracks, res);

  /* debug
  for (int i = 0; i < stracks.size(); i++) {
    vector<float> tlwh = stracks[i].tlwh;
    bool vertical      = tlwh[2] / tlwh[3] > 1.6;

    if (tlwh[2] * tlwh[3] > 20 && !vertical) {
      Scalar s = tracker->get_color(stracks[i].track_id);
      cv::putText(image, cv::format("%d", stracks[i].track_id),
                  cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6,
                  cv::Scalar(0, 0, 255), 2, LINE_AA);
      cv::rectangle(image, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s,
                    2);

      // choose to print
      if (stracks[i].track_id == 1) {
        cout << "is activated " << stracks[i].is_activated << endl;
        cout << "Track ID " << stracks[i].track_id << endl;
        cout << "State " << stracks[i].state << endl;
        cout << "Frame " << stracks[i].frame_id << endl;
        cout << "Tracklet Len " << stracks[i].tracklet_len << endl;
        cout << "Start Frame " << stracks[i].start_frame << endl;
        cout << "Score " << stracks[i].score << endl;
      }
    }
    cv::putText(image,
                cv::format("frame: %d fps: %d num: %d", num_frames,
                            num_frames * 1000000 / total_ms, stracks.size()),
                cv::Point(0, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2, LINE_AA);
  }
  */

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
void ByteTrackWrapper::draw_bbox(
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

void ByteTrackWrapper::ConvertDetectionsToInternalBoxes(
    const std::vector<jojo::perception::base::Object>& det,
    std::vector<bytetracker::Object>& in_boxes) {
  in_boxes.clear();

  /* way 1
  in_boxes.reserve(det.size());
  for (const auto& d : det) {
    jojo::perception::base::BBox2DF p = d.camera_supplement.box.Center();
    bytetracker::Object box;
    box.rect.x = p.x;
    box.rect.y = p.y;
    box.rect.width =
        d.camera_supplement.box.xmax - d.camera_supplement.box.xmin;
    box.rect.height =
        d.camera_supplement.box.ymax - d.camera_supplement.box.ymin;
    box.prob  = d.confidence;
    box.label = jojo::perception::base::BoxTypeToInt(d.type);
    in_boxes.emplace_back(std::move(box));
  }
  */

  // /* way 2
  in_boxes.resize(det.size());
  for (size_t i = 0; i < det.size(); ++i) {
    const auto& d = det[i];
    const auto& b = d.camera_supplement.box;

    auto& box       = in_boxes[i];
    box.rect.x      = b.xmin;
    box.rect.y      = b.ymin;
    box.rect.width  = b.xmax - b.xmin;
    box.rect.height = b.ymax - b.ymin;
    box.prob        = d.confidence;
    box.label       = jojo::perception::base::BoxTypeToInt(d.type);
    // std::cout << "box.label: " << box.label << std::endl;
  }
  // */
}

void ByteTrackWrapper::ConvertInternalBoxesToTrackResults(
    const std::vector<bytetracker::STrack>& in_boxes,
    std::vector<jojo::perception::base::Object>& track_results) {
  track_results.clear();

  /* way 1
  track_results.reserve(in_boxes.size());
  for (const auto& b : in_boxes) {
    jojo::perception::base::Object res;
    res.type = jojo::perception::base::SwitchBoxTypeWrapper(b.get_label());

    res.confidence = b.score;

    res.local_track_id = b.track_id;

    auto& box = res.camera_supplement.box;
    box.xmin  = b.tlwh[0];
    box.ymin  = b.tlwh[1];
    box.xmax  = b.tlwh[0] + b.tlwh[2];
    box.ymax  = b.tlwh[1] + b.tlwh[3];

    track_results.emplace_back(std::move(res));
  }
  */

  // /* way 2
  track_results.resize(in_boxes.size());
  for (size_t i = 0; i < in_boxes.size(); ++i) {
    const auto& b = in_boxes[i];

    auto& res = track_results[i];
    // 在检测起中已经映射到 自定义的 id，这里不要再映射可
    // res.type = jojo::perception::base::SwitchBoxTypeWrapper(b.get_label());
    // !! 直接基于 int 转换 对应的 枚举类型
    const int label = b.get_label();
    if (label < 0 ||
        label >= static_cast<int>(
                     jojo::perception::base::ObjectType::MAX_OBJECT_TYPE)) {
      res.type = jojo::perception::base::ObjectType::UNKNOWN;
    } else {
      res.type = static_cast<jojo::perception::base::ObjectType>(label);
    }
    // std::cout << "res.type: " << label << std::endl;

    res.confidence = b.score;

    res.camera_supplement.local_track_id = b.track_id;

    auto& box = res.camera_supplement.box;
    box.xmin  = b.tlwh[0];
    box.ymin  = b.tlwh[1];
    box.xmax  = b.tlwh[0] + b.tlwh[2];
    box.ymax  = b.tlwh[1] + b.tlwh[3];
  }
  // */
}
