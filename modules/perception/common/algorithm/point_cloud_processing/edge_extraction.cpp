#include "modules/perception/common/algorithm/point_cloud_processing/edge_extraction.h"

namespace jojo {
namespace perception {
namespace algorithm {

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
PointCloudEdgeExtractor::estimate_borders_impl(
    typename pcl::PointCloud<PointT>::Ptr& cloud, float normal_radius,
    float radius_estimation, float angle_threshold, bool show) {
  // 定义一个进行 法线估计 的对象
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimation;
  // 保存法线估计的结果
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  // 在我的工程中，点云处理前，已经去除 -nan，所以这里不需要再去除
  // 过滤无效点
  // typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);
  // std::cerr << "Original cloud size: " << cloud->size() << std::endl;
  // std::cerr << "Filtered cloud size: " << filtered_cloud->size() << std::endl;
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud = cloud;

  normal_estimation.setInputCloud(filtered_cloud);
  // 设置法线估计的半径
  normal_estimation.setRadiusSearch(normal_radius);
  // 表示计算点云法向量时，搜索的点云个数
  // normal_estimation.setKSearch(10);
  // 将法线估计结果保存至normals
  normal_estimation.compute(*normals);
  // 输出法线的个数
  std::cout << "normal_radius: " << normal_radius << std::endl;
  std::cerr << "normals: " << normals->size() << std::endl;

  // 定义一个进行 边界特征估计 的对象
  pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary>boundary_estimation;
  // 保存边界估计结果
  pcl::PointCloud<pcl::Boundary> boundaries;
  // 保存边界点的点云
  typename pcl::PointCloud<PointT>::Ptr cloud_boundary(new pcl::PointCloud<PointT>);

  // 设置输入的点云
  boundary_estimation.setInputCloud(filtered_cloud);
  // 设置边界估计的法线，边界估计依赖于法线
  boundary_estimation.setInputNormals(normals);
  // 设置边界估计所需要的半径,
  // 这里的Threadshold为一个浮点值，可取点云模型密度的10倍
  boundary_estimation.setRadiusSearch(radius_estimation);
  // 边界估计的角度阈值 M_PI / 4  并计算 k 邻域点的法线夹角, 若大于阈值则为边界特征点
  boundary_estimation.setAngleThreshold(angle_threshold);
  // 设置搜索方式KdTree
  boundary_estimation.setSearchMethod(
      typename pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>));
  // 将边界估计结果保存在boundaries
  boundary_estimation.compute(boundaries);
  std::cerr << "AngleThreshold: " << angle_threshold << std::endl;
  // 输出边界点的个数
  std::cerr << "boundaries: " << boundaries.points.size() << std::endl;

  // 存储估计为边界的点云数据，将边界结果保存为 PointT 类型
  for (int i = 0; i < filtered_cloud->points.size(); i++) {
    // std::cout << (*normals)[i].normal_x << std::endl;
    if (!pcl::isFinite(normals->points[i])) {
      continue;
    }
    /*
    if (boundaries[i].boundary_point > 0 &&
        std::abs((*normals)[i]._Normal::normal_z) < 0.1) {
      cloud_boundary->push_back(filtered_cloud->points[i]);
    }
    */
    if (boundaries[i].boundary_point > 0 &&
        std::abs(normals->points[i].normal_z) < 0.1) {
      cloud_boundary->push_back(filtered_cloud->points[i]);
    }
  }

  // Searching NN in radius R
  typename pcl::PointCloud<PointT>::Ptr cloud_boundary_filtered(new pcl::PointCloud<PointT>);
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud_boundary);

  std::vector<int> index_radius;
  std::vector<float> distance_radius;
  double search_radius = 0.5;
  // std::vector<cv::Point3f> filtered_cloud;

  for (int i = 0; i < cloud_boundary->points.size(); i++) {
    kdtree.radiusSearch(cloud_boundary->points[i], search_radius, index_radius,
                        distance_radius);
    if (index_radius.size() > 5) {
      cloud_boundary_filtered->push_back(cloud_boundary->points[i]);

      // meter to centimeter
      // clang-format off
      // filtered_cloud.push_back(cv::Point3f(cloud_boundary->points[i].x * 100,
      //                                      cloud_boundary->points[i].y * 100,
      //                                      cloud_boundary->points[i].z * 100));
      // clang-format on
    }
  }

  if (show) {
    typename boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("Boundary Extraction"));

    int viewport_1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewport_1);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, viewport_1);
    viewer->addText("Raw point clouds", 10, 10, "v1_text", viewport_1);

    viewer->addPointCloud<PointT>(filtered_cloud, "sample_cloud", viewport_1);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample_cloud",
        viewport_1);

    int viewport_2(0);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, viewport_2);
    viewer->setBackgroundColor(0.5, 0.5, 0.5, viewport_2);
    viewer->addText("Boudary point clouds", 80, 80, "v2_text", viewport_2);

    viewer->addPointCloud<PointT>(cloud_boundary_filtered, "boundary_cloud",
                                  viewport_2);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "boundary_cloud",
        viewport_2);

    // 取消坐标系
    // viewer->addCoordinateSystem(1.0);
    // 设置照相机参数，使用户从默认的角度和方向观察点云
    // viewer->initCameraParameters();

    viewer->spin();
  }

  // return filtered_cloud;
  return cloud_boundary_filtered;
}

// ⭐ 显式实例化你要用的类型（这一步很重要）
template pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudEdgeExtractor::estimate_borders_impl<pcl::PointXYZ>(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float, float, float, bool);

template pcl::PointCloud<pcl::PointXYZI>::Ptr
PointCloudEdgeExtractor::estimate_borders_impl<pcl::PointXYZI>(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float, float, float, bool);

void PointCloudEdgeExtractor::show_points(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudVisualizer(
      new pcl::visualization::PCLVisualizer);
  PointCloudVisualizer->setBackgroundColor(0, 0, 0);
  PointCloudVisualizer->addPointCloud<pcl::PointXYZRGB>(cloud);
  while (!PointCloudVisualizer->wasStopped()) {
    PointCloudVisualizer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));  // 0.1s
  };
}

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo
