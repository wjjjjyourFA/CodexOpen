#include "modules/perception/camera_location_estimation/camera_location_estimation.h"

namespace jojo {
namespace perception {
namespace cle {
namespace base = jojo::perception::base;
namespace cdss = jojo::perception::cdss;
namespace ct   = jojo::perception::ct;
// namespace tools = jojo::tools;

CameraLocationEstimation::CameraLocationEstimation(uint mode) {
  this->mode = mode;
  switch (mode) {
    case 1:  // only detection
      image_detector = std::make_shared<cdss::YoloObstacleDetector>();
      break;
    case 2:  // detection and tracking
      image_tracking = std::make_shared<ct::CameraTracking>();
      break;
    default:
      std::cerr << "CameraLocationEstimation error: no such mode" << std::endl;
      return;
      break;
  }
}

CameraLocationEstimation::~CameraLocationEstimation() {}

void CameraLocationEstimation::Init(const std::string& engine_file) {
  if (initialized_) return;

  if (!this->InitEngine(engine_file)) {
    initialized_ = false;
    return;
  }

  this->InitCluster();
  if (!object_cluster->isInited()) {
    initialized_ = false;
    return;
  }

  // segs.reserve(16);

  initialized_ = true;
}

bool CameraLocationEstimation::InitEngine(const std::string& engine_file) {
  if (initialized_) return true;

  if (mode == 1) {
    image_detector->Init(engine_file);
    if (!image_detector->isInited()) {
      return false;
    }
  } else if (mode == 2) {
    image_tracking->Init(engine_file);
    if (!image_tracking->isInited()) {
      return false;
    }
  }

  return true;
}

void CameraLocationEstimation::InitCluster() {
  // Cluster Base On FH
  object_cluster = std::make_shared<jojo::perception::lidar::ObjectCluster>();
  object_cluster->init3d(hps_.imageSize[0], hps_.imageSize[1],
                         hps_.imageSize[2]);
  object_cluster->set_params(hps_.eps, hps_.minPts);

  // clang-format off
  this->filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  this->cluster_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  /* debug
  // this->filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  this->clustered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  */
  // clang-format on
}

void CameraLocationEstimation::Start() {
  isRunning_ = true;

  if (mode == 1) {
    this->image_detector->Start();
  } else if (mode == 2) {
    this->image_tracking->Start();
  }
}

void CameraLocationEstimation::Stop() {
  isRunning_   = false;
  initialized_ = false;

  if (mode == 1) {
    this->image_detector->Stop();
  } else if (mode == 2) {
    this->image_tracking->Stop();
  }
}

void CameraLocationEstimation::Run() {
  while (isRunning_) {
    usleep(10000000);
  }
}

void CameraLocationEstimation::DetectionAndLocation(cv::Mat& cimage,
                                                    cv::Mat& projection_mask,
                                                    cv::Mat& show_image,
                                                    bool show) {
  // std::cout << "debug ----> start DetectionAndLocation" << std::endl;

  // step 1: detection
  if (image_detector == nullptr) {
    std::cerr << "image_detector is nullptr!" << std::endl;
    return;
  }

  std::vector<jojo::perception::base::Object> detections;
  image_detector->YOLO(cimage, detections, cimage.cols, cimage.rows, false);

  // way2
  auto& tracks = detections;

  // step 2: get obj rect && obj projection cloud
  std::vector<FrameObject> frame_boxes;
  this->FrameObjectGetObjRec(tracks, frame_boxes, show_image, false);

  // step 3: location
  // segs.clear();
  this->GetCloudAndCluster(frame_boxes, projection_mask, show_image);

  if (show) {
    this->DrawLocateCube(show_image, frame_boxes);
  }
}

void CameraLocationEstimation::TrackingAndLocation(cv::Mat& cimage,
                                                   cv::Mat& projection_mask,
                                                   cv::Mat& show_image,
                                                   bool show) {
  // std::cout << "debug ----> start TrackingAndLocation" << std::endl;

  // step 1: detection and tracking
  if (image_tracking == nullptr) {
    std::cerr << "image_tracking is nullptr!" << std::endl;
    return;
  }

  std::vector<jojo::perception::base::Object> tracks;
  image_tracking->DetectionAndTracking(cimage, tracks, cimage.cols, cimage.rows,
                                       false);

  // step 2: get obj rect && obj projection cloud
  std::vector<FrameObject> frame_boxes;
  this->FrameObjectGetObjRec(tracks, frame_boxes, show_image, false);

  // step 3: location
  // segs.clear();
  this->GetCloudAndCluster(frame_boxes, projection_mask, show_image);

  if (show) {
    this->DrawLocateCube(show_image, frame_boxes);
  }
}

// void CameraLocationEstimation::DetectionAndLocation(
//     std::vector<std::shared_ptr<cv::Mat>>& cimage,
//     std::vector<std::shared_ptr<cv::Mat>>& projection_mask,
//     std::vector<std::shared_ptr<cv::Mat>>& show_image) {
//   for (size_t i = 0; i < cimage.size(); i++) {
//     this->DetectionAndLocation(cimage[i], projection_mask[i], show_image[i]);
//   }
// }

void CameraLocationEstimation::FrameObjectGetObjRec(
    std::vector<base::Object>& detections,
    std::vector<FrameObject>& frame_boxes, cv::Mat& image, bool show) {
  // std::cout << "debug ----> start GetObjectRec" << std::endl;

  if (detections.empty()) {
    std::cerr << "here is no obj detect! " << std::endl;
    return;
  }

  // frame_boxes.clear();
  frame_boxes.reserve(detections.size());

  for (auto& d : detections) {
    // 筛选我需要的目标
    // 也许没必要，模型训练的类别，每个都应当是需要的才对
    if (!this->BoxTypeNeed(d.type)) {
      continue;
    }

    // way 2
    FrameObject fb;
    // fb.obj = d;
    // !! d 不再可用
    fb.obj = std::move(d);

    auto& c = fb.obj.camera_supplement.box;

    // fb.srcRec = cv::Rect(c.p[0].x, c.p[0].y, c.width, c.height);
    fb.srcRec = cv::Rect(c.xmin, c.ymin, c.xmax - c.xmin, c.ymax - c.ymin);

    // x1 和 x2 定义的是矩形的左上角点，而不是中心点。
    // OpenCV中的cv::Rect不直接支持以中心点和宽度/高度来定义矩形。它主要使用左上角点和宽度/高度来定义矩形。

    /*****************************location*********************************/
    // 计算中心矩形，检测结果框的重制，以检测框的中心点 收敛一定范围。

    fb.cenRec = AdjustRectWithScale(fb.srcRec, hps_.scale,
                                    cv::Size(image.cols, image.rows));

    if (show) {
      // 画检测框
      // clang-format off
      if (mode == 1) {
        this->image_detector->DrawObjectRec(image, fb.obj.type, fb.srcRec);
        // this->image_detector->DrawObjectRec(image, fb.obj.type, fb.cenRec);
      } else if (mode == 2) {
        auto& local_track_id = fb.obj.camera_supplement.local_track_id;
        this->image_tracking->DrawObjectRec(image, fb.obj.type, fb.srcRec, local_track_id);
        // this->image_tracking->DrawObjectRec(image, fb.obj.type, fb.cenRec, local_track_id);
      }
      // clang-format on
    }

    frame_boxes.emplace_back(std::move(fb));
  }

  // 只会显示有检测目标的图像
  if (show) {
    cv::namedWindow("FrameObject ObjRec", cv::WINDOW_NORMAL);
    cv::imshow("FrameObject ObjRec", image);
    cv::waitKey(1);
  }
}

bool CameraLocationEstimation::BoxTypeNeed(const base::ObjectType& type) {
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

void CameraLocationEstimation::GetCloudAndCluster(
    std::vector<FrameObject>& frame_boxes, cv::Mat& mask, cv::Mat& image) {
  if (frame_boxes.empty()) {
    return;
  }
  // std::cout << "frame_boxes.size(): " << frame_boxes.size() << std::endl;

  // TODO：将三个通道的 mask Mat 提前分配好，避免重复分配内存
  std::vector<cv::Mat> splits;
  cv::split(mask, splits);
  // std::cout<<"splits.size(): "<<splits.size()<<std::endl;

  // debug show 仅支持处理连续 Mat
  // show_cv_splits_cloud(splits);

  // one by one 每个检测框单独去Cluster
  // 已经提前筛选，只定位测距我需要的目标
  std::vector<cv::Mat> splits_roi;
  splits_roi.resize(3);

  /* debug
  // auto t0 = std::chrono::high_resolution_clock::now();

  // this->filtered->clear();
  this->clustered->clear();
  */

  for (size_t i = 0; i < frame_boxes.size(); i++) {
    // roi 选框 xyz
    auto& fb  = frame_boxes[i];
    auto& roi = fb.cenRec;

    // Image roi 提取 xyz 点云, 但 Cloud 是 Lidar XYZI | Radar XYZV
    // TODO：通过索引进行补齐 pcl::PointXYZ ==> pcl::PointXYZI
    RoiMask2PointCloud(splits, splits_roi, roi, filtered_cloud, hps_.RoiLimit);

    // show2d_lidar_data(filtered_cloud, 0, 0, "show_cv_rect_roi_cloud");

    // clang-format off
    /* debug
    filtered->points.insert(filtered->points.end(),
                            filtered_cloud->points.begin(),
                            filtered_cloud->points.end());
    */

    // 20251208 intensity = 0；RoiMask2PointCloud 未能生成 pcl::PointXYZI
    // std::shared_ptr<base::Segment> _pseg = std::make_shared<base::Segment>();
    // 外部不需要 std::make_shared<Segment>()，因为 _pseg 是引用传递，内部会给它赋值
    std::shared_ptr<base::Segment> _pseg = nullptr;
    this->Cluster(filtered_cloud, _pseg);
    if (!_pseg) continue; // 跳过空聚类

    // 得到聚类点云，只有一簇
    cluster_cloud = _pseg->points;
    // std::cout << "cluster_cloud.size(): " << cluster_cloud->size() << std::endl;

    // show2d_lidar_data(cluster_cloud, 0, 0, "show_roi_cluster_cloud");

    /* debug
    clustered->points.insert(clustered->points.end(),
                             cluster_cloud->points.begin(),
                             cluster_cloud->points.end());
    */
    // clang-format on

    // segs.push_back(_pseg);

    fb.obj.camera_supplement.box3d_supplement = _pseg->bbox;

    fb.obj.camera_supplement.local_center = _pseg->center;
  }

  /* debug
  // filtered->width  = filtered->points.size();
  // filtered->height = 1;
  // filtered->is_dense = false;
  // show2d_lidar_data(filtered, 0, 0, "show_cv_rect_roi_cloud");

  clustered->width    = clustered->points.size();
  clustered->height   = 1;
  clustered->is_dense = false;
  // show2d_lidar_data(clustered, 0, 0, "show_roi_cluster_cloud");

  // auto t1   = std::chrono::high_resolution_clock::now();
  // double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  // std::cout << "cost time: " << ms << " ms" << std::endl;
  */
}

void CameraLocationEstimation::Cluster(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
    std::shared_ptr<base::Segment>& result) {
  /* 这一段包含聚类,以及透视关系约束的程序,一方面输出对规划有意义的目标(能给出真实坐标的目标),
  * 另外一方面对检测到的部分结果进行透视关系的约束
  */
  if (!cloud_ptr || cloud_ptr->empty()) {
    std::cerr << " Cluster Get Null Cloud Ptr ! " << std::endl;
    return;
  }

  if (cloud_ptr->points.size() < hps_.pixel_threshold) {
    std::cerr << " Cluster Get Data < Threshold ! " << std::endl;
    return;
  }

  object_cluster->SetInputCloud(cloud_ptr);

  object_cluster->Run(1 /*MODE == RegionGrowing*/);

  // way 2 调用预设聚类策略，直接返回结果
  object_cluster->ClusterPolicy(result, 1);
  // std::cout << "ClusterPolicy Success ! " << std::endl;
  // std::cout << "result.size(): " << result->points->size() << std::endl;
}

void CameraLocationEstimation::SetProjectionMatrix(
    const Eigen::Matrix4f& projection_matrix) {
  // 取4x4的前3行4列赋值
  projection_matrix_ = projection_matrix.block<3, 4>(0, 0);
}

void CameraLocationEstimation::DrawLocateCube(
    cv::Mat& frame, std::vector<FrameObject>& frame_boxes) {
  static int width  = 1024;
  static int height = 768;
  // static int width  = 1920;
  // static int height = 1080;

  // 3d bev
  // cv::Mat LidarImage = cv::Mat::zeros(height, width, CV_8UC3);

  for (const auto& fb : frame_boxes) {
    // 2d 检测框
    cv::rectangle(frame, fb.srcRec, cv::Scalar(0, 255, 0), 2);

    // 3d 检测框
    auto& cs = fb.obj.camera_supplement;
    auto& p  = cs.box3d_supplement.corners;

    auto& local_center   = cs.local_center;
    auto& local_track_id = cs.local_track_id;

    if (local_center.isZero(1e-6)) continue;

    // 投影 3D 顶点
    std::vector<cv::Point> img_pts(8);
    for (int i = 0; i < 8; i++) {
      Eigen::Vector4f pt3d_h(p[i].x, p[i].y, p[i].z, 1.0f);  // 齐次坐标
      Eigen::Vector3f uv = this->projection_matrix_ * pt3d_h;  // 投影
      img_pts[i]         = cv::Point(static_cast<int>(uv.x() / uv.z()),
                                     static_cast<int>(uv.y() / uv.z()));
    }

    // 画立方体边框
    cv::Scalar color = base::BoxTypetoColor(fb.obj.type);

    // 画前面底面 (0-1-2-3)
    for (int i = 0; i < 4; i++)
      cv::line(frame, img_pts[i], img_pts[(i + 1) % 4], color, 2);
    // 画后面顶面 (4-5-6-7)
    for (int i = 4; i < 8; i++)
      cv::line(frame, img_pts[i], img_pts[4 + (i + 1) % 4], color, 2);
    // 画竖直边  (0-4,1-5,2-6,3-7)
    for (int i = 0; i < 4; i++)
      cv::line(frame, img_pts[i], img_pts[i + 4], color, 2);

    // 画中心点
    Eigen::Vector4f center_h(local_center.x(), local_center.y(),
                             local_center.z(), 1.0f);
    Eigen::Vector3f uv_c = this->projection_matrix_ * center_h;
    cv::Point center_pt(static_cast<int>(uv_c.x() / uv_c.z()),
                        static_cast<int>(uv_c.y() / uv_c.z()));
    cv::circle(frame, center_pt, 3, cv::Scalar(0, 0, 255), -1);

    // clang-format off
    static const float font_scale = height * 0.0009f;
    static const int thickness    = (height + width) * 0.001f;

    const std::string text_distance = cv::format("distance: %.2fm", local_center.norm());
    const std::string text_xy = cv::format("x,y: (%.2f, %.2f)", local_center.x(), local_center.y());
    const std::string text_track = cv::format("ID: %d", local_track_id);

    // 计算文本高度（取最大字体基准）
    static int text_h = -1;  // 未初始化标记
    if (text_h < 0) {
      int baseline = 0;
      cv::Size text_size = cv::getTextSize(
          "distance: 100.00m",  // 使用最长模板字符串
          cv::FONT_HERSHEY_COMPLEX,
          font_scale, thickness, &baseline);
      text_h = text_size.height;
    }

    const std::string name = base::BoxTypetoString(fb.obj.type);
    const std::string text_class = cv::format("%s : %.2f", name.c_str(), fb.obj.confidence);

    // 三段文本布局（上、中、下）
    DrawText(frame, center_pt, text_track, cv::Scalar(255, 0, 0), -(text_h * 3));   // Track ID（蓝）
    DrawText(frame, center_pt, text_class, cv::Scalar(0,0,255), -(text_h * 2)); // 类别（红）
    DrawText(frame, center_pt, text_distance, cv::Scalar(0,255,0), -text_h);   // 距离（绿）
    DrawText(frame, center_pt, text_xy, cv::Scalar(0,255,0), +text_h);   // 坐标（绿）
    // clang-format on

    // 3d bev
    // show2d_lidar_bev(p, local_center, color, &LidarImage);
  }

  /* debug
  show2d_lidar_data(clustered, 0, 1, "show_roi_cluster_cloud", &LidarImage);
  */

  bool show = true;
  if (show) {
    cv::namedWindow("FrameObject ObjRec", cv::WINDOW_NORMAL);
    cv::imshow("FrameObject ObjRec", frame);
    cv::resizeWindow("FrameObject ObjRec", width, height);
    // cv::resizeWindow("FrameObject ObjRec", 1024, 768);
    cv::waitKey(1);
  }
}

}  // namespace cle
}  // namespace perception
}  // namespace jojo
