#include "modules/mapping/map_visualization/map_visualization.h"

/* pipeline：生成完整的 label 底图
// 循环开始前执行
this->FillUnknownGroundByHeight();

// 每帧运行
this->GenerateSequencePassableArea();
std::cout << "Generating passable area for pose sequence " << std::endl;

// 整个循环结束后，再对 mat 进行补充
FillHoleSemanticMap(label_terrain_mat);
std::cout << "Filling holes Ok" << std::endl;

this->SaveLabelTerrainMat();
*/

MapVisualization::MapVisualization() {
  terrain_map = std::make_shared<TerrainMap>();
  frame.reset(new pcl::PointCloud<pcl::PointXYZI>);
  frame->points.reserve(50000);
}

MapVisualization::~MapVisualization() {}

void MapVisualization::Init(
    std::shared_ptr<jojo::mapping::RuntimeConfig> rparam,
    std::shared_ptr<jojo::mapping::StaticConfig> sparam) {
  rparam_ = rparam;
  sparam_ = sparam;
  this->SetDataFolder();
  // 3d map
  // this->LoadRawMap(rparam_->map_file_path);

  // 固有属性
  hps_.map_resolution = sparam_->map_resolution;
  // 本代码要生成的数据的相关参数
  hps_.half_length     = rparam_->half_length;
  hps_.max_search_dist = rparam_->max_search_dist;

  // 2d map

  // terrain_map
  terrain_map->SetInpaintedFlag(true);
  terrain_map->Init(this->terrain_map_dir, sparam);

  p_line_width = rparam_->line_width;
}

void MapVisualization::LoadRawMap(const std::string& map_path) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  double load_start = omp_get_wtime();
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *raw_cloud) == -1) {
    std::cerr << "Couldn't read map file from " << map_path;
    abort();
  }
  double load_end = omp_get_wtime();

  std::cout << "Loaded map " << raw_cloud->points.size()
            << " points from " + map_path;
  std::cout << "Load map cost: " << (load_end - load_start) * 1000 << "ms";

  this->SetRawMap(raw_cloud);
}

void MapVisualization::SetRawMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& map) {
  this->map_ = map;
  std::cout << "map size: " << map->points.size() << std::endl;
}

void MapVisualization::SetPoseCenter(const Eigen::Vector3d& p_center) {
  pose_center = p_center;
}

void MapVisualization::SetDataFolder() {
  this->prefix  = rparam_->root_path + "/" + rparam_->file_name;
  this->postfix = this->prefix + "-O";

  // 栅格地图
  this->terrain_map_dir = this->postfix + "/tmp_data/terrain_modeling_offline/";

  // clang-format off
  this->xml_path  = this->postfix + "/label_data/map2d/" + "label.xml";
  this->gray_path  = this->postfix + "/label_data/map2d/" + "label_gray.png";
  this->color_path = this->postfix + "/label_data/map2d/" + "label_color.png";
  // clang-format on
}

void MapVisualization::LoadMapLabel() {
  // 读取 人工标注 的标签
  // sparam 已经读取 XML 对应地图的配置参数

  // 1. 初始化坐标转换参数
  // clang-format off
  // height, width <==> terrain_map.data.rows, cols
  label_terrain_mat = cv::Mat::zeros(sparam_->height, sparam_->width, CV_32SC1);
  // clang-format on

  // 2. 读取 XML（逐行解析并转换坐标）
  std::ifstream fin(this->xml_path);
  if (!fin.is_open()) {
    std::cerr << "[LoadLabel] Failed to open xml: " << this->xml_path
              << std::endl;
    return;
  }

  std::string line;
  while (std::getline(fin, line)) {
    bool is_polyline = (line.find("<polyline") != std::string::npos &&
                        line.find("label=\"pl\"") != std::string::npos);
    bool is_polygon  = (line.find("<polygon") != std::string::npos &&
                        line.find("label=\"pg\"") != std::string::npos);

    if (is_polyline || is_polygon) {
      auto ppos = line.find("points=\"");
      if (ppos == std::string::npos) continue;

      ppos += 8;
      auto pend = line.find("\"", ppos);
      if (pend == std::string::npos) continue;

      std::string points_str = line.substr(ppos, pend - ppos);
      // 解析 CVAT 格式的坐标点
      std::vector<cv::Point> pts_xml = ParseCVATPoints(points_str);

      // 坐标转换：CVAT XML ==> OPENCV MAT
      // 地图 是用 opencv mat 存的
      std::vector<cv::Point> pts_target;
      pts_target.reserve(pts_xml.size());
      for (const auto& p : pts_xml) {
        pts_target.push_back(XMLToMatIndex(p, label_terrain_mat.rows));
      }

      if (is_polyline && pts_target.size() >= 2) {
        // 绘制折线
        for (size_t i = 1; i < pts_target.size(); ++i) {
          cv::line(label_terrain_mat, pts_target[i - 1], pts_target[i],
                   cv::Scalar(SemanticLabel::ROAD_BRIDGE), p_line_width,
                   cv::LINE_8);
        }
      } else if (is_polygon && pts_target.size() >= 3) {
        // 绘制多边形
        const std::vector<std::vector<cv::Point>> polys{pts_target};
        cv::polylines(label_terrain_mat, polys, true,
                      cv::Scalar(SemanticLabel::ROAD_BRIDGE), p_line_width,
                      cv::LINE_8);
      }
    }
  }
  fin.close();

  std::cout << "[LoadLabel] label_terrain_mat generated with transform. size = "
            << label_terrain_mat.cols << " x " << label_terrain_mat.rows
            << std::endl;

  // this->DebugShow();
}

void MapVisualization::FillUnknownGroundByHeight() {
  // 依据栅格图，进行预处理：对于有有效高度值的栅格，如果当前语义是 UNKNOWN，则改成 GROUND
  for (int r = 0; r < label_terrain_mat.rows; ++r) {
    for (int c = 0; c < label_terrain_mat.cols; ++c) {
      double height = terrain_map->GetValueFromRC(r, c);
      if (height == INVALID_VALUE) continue;

      int& label = label_terrain_mat.at<int>(r, c);
      if (label == SemanticLabel::UNKNOWN) {
        label = SemanticLabel::GROUND;
      }
    }
  }
  std::cout << "FillUnknownGroundByHeight Ok" << std::endl;

  // this->DebugShow();
}

void MapVisualization::GenerateSequencePassableArea(
    const Eigen::Matrix4f& in_pose) {
  // std::cout << "GenerateSequencePassableArea" << std::endl;
  // std::cout << std::fixed << std::setprecision(9) << std::endl;

  // 世界坐标 3D convert to 2D 世界坐标
  Eigen::Matrix4f pose = in_pose;
  if (sparam_->b_use_pose_center) {
    pose.block<3, 1>(0, 3) -= pose_center.cast<float>();
  }

  Eigen::Vector3d ypr;
  Eigen::Matrix3d R = pose.topLeftCorner<3, 3>().cast<double>();
  jojo::common::transform::RotationToEulerZYX(R, ypr);
  double yaw = ypr[0];
  // map 坐标 (m)
  Eigen::Vector2d pos = pose.block<2, 1>(0, 3).cast<double>();
  // std::cout << pos.x() << " " << pos.y() << std::endl;

  // 沿着车辆中心线，在地图上“横向扫描”，把可达区域标成 ROAD，遇到边界就停止扩展。
  // 计算车体当前朝向
  //        ↑ forward
  //        |
  // left ← + → right
  Eigen::Vector2d dir_forward(std::cos(yaw), std::sin(yaw));
  Eigen::Vector2d dir_left(-std::sin(yaw), std::cos(yaw));
  Eigen::Vector2d dir_right = -dir_left;

  // 地图分辨率（m）
  const double step_m = hps_.map_resolution;
  const int max_k =
      static_cast<int>(hps_.max_search_dist / hps_.map_resolution);

  // 考虑到栅格图的分辨率，步长取栅格图分辨率
  for (double s = -hps_.half_length; s <= hps_.half_length; s += step_m) {
    Eigen::Vector2d base_xy = pos + s * dir_forward;

    // 向左延伸
    for (int k = 0; k < max_k; ++k) {
      // 得到 世界坐标
      Eigen::Vector2d p = base_xy + k * step_m * dir_left;

      // 用世界坐标查询栅格图信息
      int r, c;
      if (terrain_map->GetValueFromXY(p.x(), p.y(), r, c) == INVALID_VALUE) {
        break;
      }

      int& label = label_terrain_mat.at<int>(r, c);
      // 邻域检查，防止穿透单像素边界
      if (IsNearBoundary(label_terrain_mat, r, c)) {
        if (label != SemanticLabel::ROAD_BRIDGE) {
          label = SemanticLabel::ROAD_SURFACE;
        }
        break;
      }
      // 属性生长
      if (label != SemanticLabel::ROAD_SURFACE) {
        label = SemanticLabel::ROAD_SURFACE;
      }
    }

    // 向右延伸
    for (int k = 1; k < max_k; ++k) {
      Eigen::Vector2d p = base_xy + k * step_m * dir_right;

      int r, c;
      if (terrain_map->GetValueFromXY(p.x(), p.y(), r, c) == INVALID_VALUE) {
        break;
      }

      int& label = label_terrain_mat.at<int>(r, c);
      if (IsNearBoundary(label_terrain_mat, r, c)) {
        if (label != SemanticLabel::ROAD_BRIDGE) {
          label = SemanticLabel::ROAD_SURFACE;
        }
        break;
      }
      if (label != SemanticLabel::ROAD_SURFACE) {
        label = SemanticLabel::ROAD_SURFACE;
      }
    }
  }

  // /* draw pose traj
  // std::cout << "traj GetValueFromXY() " << std::endl;
  // cv::circle(label_terrain_mat, cv::Point(c, r), 1, cv::Scalar(0, 0, 255), -1);
  int r, c;
  if (terrain_map->GetValueFromXY(pos.x(), pos.y(), r, c) == INVALID_VALUE) {
    return;
  }
  // std::cout << "r, c = " << r << " " << c << std::endl;
  int& label = label_terrain_mat.at<int>(r, c);
  // 直接给语义，通过 label_terrain_mat 生成颜色图
  label = SemanticLabel::OBSTACLE;
  // std::cout << "traj GetValueFromXY() end" << std::endl;
  // */

  // this->DebugShow();
}

void MapVisualization::FillHoleMap() {
  // 整个生长过程结束后，再对 mat 进行补充
  FillHoleSemanticMap(label_terrain_mat);
  std::cout << "Filling holes Ok" << std::endl;
}

void MapVisualization::SaveLabelTerrainMat() {
  if (label_terrain_mat.empty()) {
    std::cerr << "[SaveLabelTerrainMat] label_terrain_mat is empty!"
              << std::endl;
    return;
  }

  if (rparam_->b_generate_label_gray) {
    // 2. 单通道灰度图（值继承 label_terrain_mat）
    // label_terrain_mat 是 CV_32SC1
    // 直接转成 CV_8UC1（假设 label 值不大：0 / 1 / 2）
    cv::Mat gray_u8;
    label_terrain_mat.convertTo(gray_u8, CV_8UC1);

    if (!cv::imwrite(gray_path, gray_u8)) {
      std::cerr << "[SaveLabelTerrainMat] Failed to save gray image: "
                << gray_path << std::endl;
    } else {
      std::cout << "[SaveLabelTerrainMat] Gray label saved: " << gray_path
                << std::endl;
    }
  }

  if (rparam_->b_generate_label_color) {
    // 3. 彩色映射图
    // 0 -> 黑色  1 -> 红色  2 -> 绿色
    // 默认黑色
    cv::Mat color_img(label_terrain_mat.rows, label_terrain_mat.cols, CV_8UC3,
                      cv::Scalar(0, 0, 0));

    for (int r = 0; r < label_terrain_mat.rows; ++r) {
      for (int c = 0; c < label_terrain_mat.cols; ++c) {
        int& label = label_terrain_mat.at<int>(r, c);

        cv::Vec3b& pix = color_img.at<cv::Vec3b>(r, c);

        pix = GetLabelColor(label);
      }
    }

    if (!cv::imwrite(color_path, color_img)) {
      std::cerr << "[SaveLabelTerrainMat] Failed to save color image: "
                << color_path << std::endl;
    } else {
      std::cout << "[SaveLabelTerrainMat] Color label saved: " << color_path
                << std::endl;
    }
  }
}

void MapVisualization::DebugShow() {
  static bool first_run = true;
  if (first_run) {
    cv::namedWindow("label_terrain_mat", cv::WINDOW_NORMAL);
    cv::resizeWindow("label_terrain_mat", 1920, 1080);
    first_run = false;
  }

  // /* debug show
  cv::Mat vis(label_terrain_mat.size(), CV_8UC3);
  for (int r = 0; r < label_terrain_mat.rows; ++r) {
    const int* src = label_terrain_mat.ptr<int>(r);

    cv::Vec3b* dst = vis.ptr<cv::Vec3b>(r);

    for (int c = 0; c < label_terrain_mat.cols; ++c) {
      dst[c] = GetLabelColor(src[c]);
    }
  }
  cv::Mat vis_resized;
  cv::resize(vis, vis_resized, cv::Size(1920, 1080), 0, 0, cv::INTER_NEAREST);
  cv::imshow("label_terrain_mat", vis_resized);
  cv::waitKey(0);
  // */
}

/* pipeline：在点云里显示各标注信息：路边-label_terrain_map
  1. 获取当前 pose，与地图坐标系对齐
  2. 得到 pose 对应的像素坐标 (r, c) ，取对应范围的 local_label_patch
  3. 通过映射关系，从栅格地图中获取到对应的点云
*/
// TODO：栅格地图 “terrain_height.bin” 中获取，改为 “地形图.tif” 或 “map.pcd” 
void MapVisualization::Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_frame,
                           const Eigen::Matrix4f& in_pose) {
  frame->clear();

  // 1. 当前帧位姿（map <- local）
  Eigen::Matrix4f pose = in_pose;
  if (sparam_->b_use_pose_center) {
    pose.block<3, 1>(0, 3) -= pose_center.cast<float>();
  }

  Eigen::Vector3d ypr;
  Eigen::Matrix3d R = pose.topLeftCorner<3, 3>().cast<double>();
  jojo::common::transform::RotationToEulerZYX(R, ypr);
  Eigen::Vector2d pos   = pose.block<2, 1>(0, 3).cast<double>();
  Eigen::Matrix3d R_m_l = jojo::common::transform::YPR2RotationZYX(ypr);
  Eigen::Vector3d T_m_l = pose.block<3, 1>(0, 3).cast<double>();
  Eigen::Matrix3d R_l_m = R_m_l.transpose();

  /* way 2
  if (!frame_world) {
    frame_world.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  frame_world->clear();
  frame_world->points.reserve(frame->size());
  pcl::transformPointCloud(*frame, *frame_world, pose);
  */

  // 2. 查询中心 rc
  int cr, cc;
  if (terrain_map->GetValueFromXY(pos.x(), pos.y(), cr, cc) == INVALID_VALUE) {
    // std::cout << "Failed to get rc from xy" << std::endl;
    return;
  }
  // std::cout << "cr: " << cr << " cc: " << cc << std::endl;

  // 此处计算的是在栅格图上的搜索半径：实际要看 100m 以内的点云 ==>
  // 100m / 0.2m = 500个栅格
  int search_radius_cell =
      static_cast<int>(hps_.search_radius / hps_.map_resolution);

  // 4. 遍历 rc 邻域
  for (int dr = -search_radius_cell; dr <= search_radius_cell; ++dr) {
    for (int dc = -search_radius_cell; dc <= search_radius_cell; ++dc) {
      int r = cr + dr;
      int c = cc + dc;

      // rc 边界
      if (r < 0 || r >= label_terrain_mat.rows || c < 0 ||
          c >= label_terrain_mat.cols) {
        continue;
      }

      // 只取 路面 点
      int label = label_terrain_mat.at<int>(r, c);
      bool need = (label == SemanticLabel::ROAD_BRIDGE ||
                   label == SemanticLabel::ROAD_SURFACE);
      if (!need) {
        continue;
      }

      // 获取 点 的 map 坐标
      double x_map, y_map;
      double z_map = terrain_map->GetValueFromRC(r, c, x_map, y_map);
      if (z_map == double(INVALID_VALUE)) {
        continue;
      }

      Eigen::Vector3d p_map(x_map, y_map, z_map);
      // 半径精确筛选（圆）
      if ((p_map.head<2>() - T_m_l.head<2>()).norm() > hps_.search_radius)
        continue;

      // 6. map -> local
      Eigen::Vector3d p_local = R_l_m * (p_map - T_m_l);

      // 7. 写入点云
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(p_local.x());
      pt.y = static_cast<float>(p_local.y());
      pt.z = static_cast<float>(p_local.z());
      // use intensity to store label
      pt.intensity = label;

      frame->push_back(pt);
    }
  }

  frame->width    = frame->size();
  frame->height   = 1;
  frame->is_dense = false;

  this->VisLabelCloud(in_frame, frame);
}

void MapVisualization::InitViewer() {
  if (vis_inited_) return;

  vis_.reset(new pcl::visualization::PCLVisualizer("Map Viewer"));
  vis_->setBackgroundColor(0, 0, 0);
  vis_->initCameraParameters();
  vis_->setCameraPosition(-50, -50, 200,  // 相机位置（原点上方）
                          0, 0, 0,  // 看向原点
                          0, 1, 0  // up方向
  );
  vis_inited_ = true;
}

void MapVisualization::VisLabelCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_raw,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_label) {
  if (!vis_inited_) return;

  vis_->removeAllPointClouds();
  vis_->removeAllShapes();

  // 原始点云：灰色
  // clang-format off
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> raw_color(cloud_raw, 180, 180, 180);
  vis_->addPointCloud(cloud_raw, raw_color, "raw_cloud");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "raw_cloud");
  // clang-format on

  // label 点云：黄色
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_rgb->reserve(cloud_label->size());

  for (const auto& pt : cloud_label->points) {
    pcl::PointXYZRGB p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;

    int label   = static_cast<int>(pt.intensity);
    cv::Vec3b t = GetLabelColor(label);

    p.r = t.val[2];
    p.g = t.val[1];
    p.b = t.val[0];

    cloud_rgb->push_back(p);
  }

  cloud_rgb->width    = cloud_rgb->size();
  cloud_rgb->height   = 1;
  cloud_rgb->is_dense = false;

  // 可视化
  // clang-format off
  vis_->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, "label_cloud");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "label_cloud");
  // clang-format on

  // 刷新显示
  vis_->spinOnce(10);
}

bool MapVisualization::GetPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  cloud = this->frame;
  return true;
}
