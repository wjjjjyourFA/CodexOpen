#include "modules/perception/camera_location_estimation/camera_location_estimation.h"

#include <unistd.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <utility>

namespace jojo {
namespace perception {
namespace cle {
namespace base = jojo::perception::base;
namespace cdss = jojo::perception::cdss;
namespace ct   = jojo::perception::ct;

CameraLocationEstimation::CameraLocationEstimation(InferenceMode mode)
    : mode_(mode) {
  if (mode_ == InferenceMode::kDetection) {
    image_detector = std::make_shared<cdss::YoloObstacleDetector>();
  } else if (mode_ == InferenceMode::kTracking) {
    image_tracking = std::make_shared<ct::CameraTracking>();
  }
}

CameraLocationEstimation::CameraLocationEstimation(unsigned int mode)
    : CameraLocationEstimation(static_cast<InferenceMode>(mode)) {}

CameraLocationEstimation::~CameraLocationEstimation() { Stop(); }

void CameraLocationEstimation::Init(const std::string& engine_file) {
  std::string error;
  if (!Initialize(engine_file, ImageLocationHyperparams(), &error)) {
    std::cerr << "CameraLocationEstimation initialization failed: " << error
              << std::endl;
  }
}

bool CameraLocationEstimation::Initialize(
    const std::string& engine_file, const ImageLocationHyperparams& config,
    std::string* error) {
  if (initialized_) return true;
  if (engine_file.empty()) return Fail("engine file must not be empty", error);
  if (!config.Validate(error)) return false;
  if (mode_ != InferenceMode::kDetection && mode_ != InferenceMode::kTracking) {
    return Fail("invalid inference mode", error);
  }

  hps_ = config;
  if (!InitEngine(engine_file))
    return Fail("detector/tracker engine initialization failed", error);

  // 聚类容量由第一帧实际 projection mask 尺寸决定，不再预设 1920x1080。
  object_cluster.reset();
  cluster_mask_size_ = cv::Size();

  if (!filtered_cloud)
    filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  if (!cluster_cloud)
    cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

  initialized_ = true;
  if (error) error->clear();

  return true;
}

bool CameraLocationEstimation::InitEngine(const std::string& engine_file) {
  if (mode_ == InferenceMode::kDetection) {
    if (!image_detector) return false;
    image_detector->Init(engine_file);
    return image_detector->isInited();
  } else if (mode_ == InferenceMode::kTracking) {
    if (!image_tracking) return false;
    image_tracking->Init(engine_file);
    return image_tracking->isInited();
  }
  return false;
}

bool CameraLocationEstimation::InitCluster(const cv::Size& mask_size) {
  if (mask_size.width <= 0 || mask_size.height <= 0 ||
      mask_size.width > std::numeric_limits<int>::max() / mask_size.height) {
    return false;
  }
  if (object_cluster && object_cluster->isInited() &&
      cluster_mask_size_ == mask_size) {
    object_cluster->set_params(hps_.eps, hps_.minPts);
    return true;
  }

  try {
    auto cluster = std::make_shared<lidar::ObjectCluster>();
    cluster->init3d(mask_size.width, mask_size.height, 1);
    cluster->set_params(hps_.eps, hps_.minPts);
    if (!cluster->isInited()) return false;
    object_cluster     = std::move(cluster);
    cluster_mask_size_ = mask_size;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

void CameraLocationEstimation::Start() {
  if (!initialized_ || isRunning_) return;
  isRunning_ = true;
  if (mode_ == InferenceMode::kDetection)
    image_detector->Start();
  else
    image_tracking->Start();
}

void CameraLocationEstimation::Stop() {
  if (!isRunning_.exchange(false)) return;
  if (mode_ == InferenceMode::kDetection && image_detector)
    image_detector->Stop();
  else if (image_tracking)
    image_tracking->Stop();
  // 底层 detector/tracker 的 Stop() 会清除其初始化状态，保持外层状态一致。
  initialized_ = false;
}

void CameraLocationEstimation::Run() {
  while (isRunning_) usleep(10000000);
}

bool CameraLocationEstimation::Estimate(cv::Mat& image,
                                        const cv::Mat& projection_mask,
                                        LocationEstimateResult* result) {
  if (!result) return false;
  result->Clear();

  if (!initialized_ || !isRunning_)
    return Fail("location estimator is not running", &result->error);
  if (image.empty() || image.type() != CV_8UC3)
    return Fail("camera image must be CV_8UC3", &result->error);
  if (projection_mask.empty() || projection_mask.type() != CV_32FC3)
    return Fail("projection mask must be CV_32FC3", &result->error);
  if (projection_mask.size() != image.size())
    return Fail("projection mask and image sizes differ", &result->error);

  if (!InitCluster(projection_mask.size()))
    return Fail("point-cloud cluster initialization failed", &result->error);

  std::vector<base::Object> detections;
  if (mode_ == InferenceMode::kDetection) {
    image_detector->YOLO(image, detections, image.cols, image.rows, false);
  } else {
    image_tracking->DetectionAndTracking(image, detections, image.cols,
                                         image.rows, false);
  }

  auto& tracks = detections;

  result->inference_count = tracks.size();
  FrameObjectGetObjRec(tracks, &result->objects, image.size());

  result->accepted_count = result->objects.size();
  result->located_count = GetCloudAndCluster(&result->objects, projection_mask);

  return true;
}

void CameraLocationEstimation::DetectionAndLocation(cv::Mat& image,
                                                    cv::Mat& projection_mask,
                                                    cv::Mat& show_image,
                                                    bool show) {
  LocationEstimateResult result;
  if (!Estimate(image, projection_mask, &result)) {
    std::cerr << result.error << std::endl;
    return;
  }
  if (show) Visualize(show_image, result);
}

void CameraLocationEstimation::TrackingAndLocation(cv::Mat& image,
                                                   cv::Mat& projection_mask,
                                                   cv::Mat& show_image,
                                                   bool show) {
  DetectionAndLocation(image, projection_mask, show_image, show);
}

void CameraLocationEstimation::FrameObjectGetObjRec(
    std::vector<base::Object>& detections,
    std::vector<FrameObject>* frame_boxes, const cv::Size& image_size) {
  if (!frame_boxes) return;

  frame_boxes->clear();
  frame_boxes->reserve(detections.size());

  for (auto& detection : detections) {
    // 筛选我需要的目标
    // 也许没必要，模型训练的类别，每个都应当是需要的才对
    if (!BoxTypeNeed(detection.type)) continue;

    FrameObject object;
    object.obj = std::move(detection);

    const auto& box = object.obj.camera_supplement.box;

    // x1 和 x2 定义的是矩形的左上角点，而不是中心点。
    // OpenCV中的cv::Rect不直接支持以中心点和宽度/高度来定义矩形。它主要使用左上角点和宽度/高度来定义矩形。
    object.srcRec =
        cv::Rect(static_cast<int>(box.xmin), static_cast<int>(box.ymin),
                 static_cast<int>(box.xmax - box.xmin),
                 static_cast<int>(box.ymax - box.ymin));
    object.srcRec &= cv::Rect(0, 0, image_size.width, image_size.height);
    if (object.srcRec.empty()) continue;

    // 计算中心矩形，检测结果框的重制，以检测框的中心点 收敛一定范围。
    object.cenRec = AdjustRectWithScale(object.srcRec, hps_.scale, image_size);
    if (!object.cenRec.empty()) frame_boxes->emplace_back(std::move(object));
  }
}

bool CameraLocationEstimation::BoxTypeNeed(const base::ObjectType& type) const {
  // clang-format off
  static const std::set<base::ObjectType> kNeededTypes = {
    base::ObjectType::UNKNOWN, 
    base::ObjectType::TREE,
    base::ObjectType::GRASS, 

    base::ObjectType::OILBOX,
    base::ObjectType::TRACK_ABATIS, 

    base::ObjectType::PERSON_ARMY,
    base::ObjectType::VEHICLE_ARMY, 

    base::ObjectType::TANK,
    base::ObjectType::TRUCK
  };
  // clang-format on

  return kNeededTypes.count(type) > 0;
}

std::size_t CameraLocationEstimation::GetCloudAndCluster(
    std::vector<FrameObject>* frame_boxes, const cv::Mat& mask) {
  if (!frame_boxes) return 0U;

  std::size_t located = 0U;
  for (auto& object : *frame_boxes) {
    // 公共函数直接遍历 ROI，不再对整张 mask 做 cv::split。
    if (RoiMask2PointCloud(mask, object.cenRec, filtered_cloud) <
        static_cast<std::size_t>(hps_.pixel_threshold)) {
      continue;
    }

    // show2d_lidar_data(filtered_cloud, 0, 0, "show_cv_rect_roi_cloud");

    std::shared_ptr<base::Segment> segment;
    Cluster(filtered_cloud, &segment);
    if (!segment) continue;

    cluster_cloud = segment->points;

    // show2d_lidar_data(cluster_cloud, 0, 0, "show_roi_cluster_cloud");
    object.obj.camera_supplement.box3d_supplement = segment->bbox;
    object.obj.camera_supplement.local_center     = segment->center;

    object.located = true;
    ++located;
  }
  return located;
}

void CameraLocationEstimation::Cluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::shared_ptr<base::Segment>* result) {
  /* 这一段包含聚类,以及透视关系约束的程序,
  * 1. 输出对规划有意义的目标(能给出真实坐标的目标),
  * 2. 对检测到的部分结果进行透视关系的约束
  */

  if (!result) return;

  result->reset();

  if (!cloud || cloud->size() < static_cast<std::size_t>(hps_.pixel_threshold))
    return;

  object_cluster->SetInputCloud(cloud);

  object_cluster->Run(1);

  // way 2 调用预设聚类策略，直接返回结果
  object_cluster->ClusterPolicy(*result, 1);
}

bool CameraLocationEstimation::SetProjectionMatrix(
    const Eigen::Matrix4f& projection_matrix) {
  if (!projection_matrix.allFinite()) return false;
  projection_matrix_ = projection_matrix.block<3, 4>(0, 0);
  projection_ready_  = true;
  return true;
}

void CameraLocationEstimation::Visualize(cv::Mat& image,
                                         const LocationEstimateResult& result) {
  DrawLocateCube(image, result.objects);
}

void CameraLocationEstimation::DrawLocateCube(
    cv::Mat& frame, const std::vector<FrameObject>& frame_boxes) {
  if (frame.empty()) return;

  for (const auto& object : frame_boxes) {
    cv::rectangle(frame, object.srcRec, cv::Scalar(0, 255, 0), 2);
    if (!object.located || !projection_ready_) continue;

    // camera 检测框
    const auto& supplement = object.obj.camera_supplement;
    // 3d 检测框
    const auto& corners = supplement.box3d_supplement.corners;

    std::vector<cv::Point> points(8);
    bool valid = true;

    // 投影 3D 顶点
    for (int i = 0; i < 8; ++i) {
      valid = ProjectPoint(
          projection_matrix_,
          Eigen::Vector3f(corners[i].x, corners[i].y, corners[i].z),
          hps_.projection_epsilon, &points[i]);
      if (!valid) break;
    }

    cv::Point center;
    valid = valid && ProjectPoint(projection_matrix_, supplement.local_center,
                                  hps_.projection_epsilon, &center);
    if (!valid) continue;

    const cv::Scalar color = base::BoxTypetoColor(object.obj.type);

    // 画立方体边框
    for (int i = 0; i < 4; ++i) {
      // 画前面底面 (0-1-2-3)
      cv::line(frame, points[i], points[(i + 1) % 4], color, 2);
      // 画后面顶面 (4-5-6-7)
      cv::line(frame, points[i + 4], points[4 + (i + 1) % 4], color, 2);
      // 画竖直边  (0-4,1-5,2-6,3-7)
      cv::line(frame, points[i], points[i + 4], color, 2);
    }
    // 画中心点
    cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), -1);

    const float font_scale = 0.7f;
    const int thickness    = 1;

    const std::string name = base::BoxTypetoString(object.obj.type);

    // clang-format off
    DrawText(frame, center, cv::format("ID: %d", supplement.local_track_id), cv::Scalar(255, 0, 0), -54);
    DrawText(frame, center, cv::format("%s : %.2f", name.c_str(), object.obj.confidence), cv::Scalar(0, 0, 255), -36);
    DrawText(frame, center, cv::format("distance: %.2fm", supplement.local_center.norm()), cv::Scalar(0, 255, 0), -18);
    // clang-format on

    cv::putText(frame,
                cv::format("x,y: (%.2f, %.2f)", supplement.local_center.x(),
                           supplement.local_center.y()),
                center + cv::Point(0, 18), cv::FONT_HERSHEY_COMPLEX, font_scale,
                cv::Scalar(0, 255, 0), thickness);
  }
}

}  // namespace cle
}  // namespace perception
}  // namespace jojo
