#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <ars548_process/Object.h>
#include <ars548_process/ObjectList.h>
#include <ars548_process/Detection.h>
#include <ars548_process/DetectionList.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/ChannelFloat32.h>

#include "config/config_manager.h"
#include "config/runtime_config.h"

using namespace jojo::drivers;

class InfoConverNode {
 public:
  InfoConverNode() {};
  ~InfoConverNode() {};

  // Get pointcloud2 iterators
  using PointCloud2Iterator = sensor_msgs::PointCloud2Iterator<float>;

  bool init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<RuntimeConfig> param);
  void run();

  void ObjectReceive(const ars548_process::ObjectList& msg);
  void DetectionReceive(const ars548_process::DetectionList& msg);

  void stop() { running_ = false; }

 protected:
  std::shared_ptr<RuntimeConfig> param_;
  sensor_msgs::PointCloud2 cloud_;
  void CreatPointCloud2();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber obj_list_sub;
  ros::Subscriber det_list_sub;

  ros::Publisher objects_marker_pub;
  ros::Publisher detections_cloud_pub;

  bool running_ = false;
};