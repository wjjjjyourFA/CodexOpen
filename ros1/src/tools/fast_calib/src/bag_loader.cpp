#include "fast_calib_ros1/bag_loader.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#if defined(__has_include)
#if __has_include(<livox_ros_driver/CustomMsg.h>)
#include <livox_ros_driver/CustomMsg.h>
#else
#include "fast_calib_ros1/legacy_livox_custom_msg.h"
#endif
#if __has_include(<livox_ros_driver2/CustomMsg.h>)
#include <livox_ros_driver2/CustomMsg.h>
#else
#include <livox_ros_driver2-1.2.4/CustomMsg.h>
#endif
#else
#include "fast_calib_ros1/legacy_livox_custom_msg.h"
#include <livox_ros_driver2-1.2.4/CustomMsg.h>
#endif

namespace jojo {
namespace tools {
namespace fast_calib {
namespace ros1 {
namespace {

const sensor_msgs::PointField* FindField(
    const sensor_msgs::PointCloud2& message,
    const std::string& name) {
  for (const auto& field : message.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

bool HostIsBigEndian() {
  const std::uint16_t value = 0x0102;
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
  return bytes[0] == 0x01;
}

template <typename Value>
Value ReadValue(const std::uint8_t* data, bool data_is_big_endian) {
  Value value;
  std::memcpy(&value, data, sizeof(Value));
  if (data_is_big_endian != HostIsBigEndian() && sizeof(Value) > 1) {
    auto* bytes = reinterpret_cast<std::uint8_t*>(&value);
    std::reverse(bytes, bytes + sizeof(Value));
  }
  return value;
}

bool ReadNumber(const std::uint8_t* point,
                const sensor_msgs::PointField& field,
                bool is_big_endian,
                double* value) {
  const std::uint8_t* data = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
      *value = ReadValue<std::int8_t>(data, is_big_endian);
      return true;
    case sensor_msgs::PointField::UINT8:
      *value = ReadValue<std::uint8_t>(data, is_big_endian);
      return true;
    case sensor_msgs::PointField::INT16:
      *value = ReadValue<std::int16_t>(data, is_big_endian);
      return true;
    case sensor_msgs::PointField::UINT16:
      *value = ReadValue<std::uint16_t>(data, is_big_endian);
      return true;
    case sensor_msgs::PointField::INT32:
      *value = ReadValue<std::int32_t>(data, is_big_endian);
      return true;
    case sensor_msgs::PointField::UINT32:
      *value = ReadValue<std::uint32_t>(data, is_big_endian);
      return true;
    case sensor_msgs::PointField::FLOAT32:
      *value = ReadValue<float>(data, is_big_endian);
      return true;
    case sensor_msgs::PointField::FLOAT64:
      *value = ReadValue<double>(data, is_big_endian);
      return true;
    default:
      return false;
  }
}

bool FieldFitsPoint(const sensor_msgs::PointField& field,
                    std::uint32_t point_step) {
  std::uint32_t size = 0;
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      size = 1;
      break;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      size = 2;
      break;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      size = 4;
      break;
    case sensor_msgs::PointField::FLOAT64:
      size = 8;
      break;
    default:
      return false;
  }
  return field.count > 0 && field.offset + size <= point_step;
}

bool AppendPointCloud2(const sensor_msgs::PointCloud2& message,
                       pcl::PointCloud<PointXYZRing>::Ptr cloud,
                       LidarType* lidar_type,
                       std::string* error) {
  const auto* x_field = FindField(message, "x");
  const auto* y_field = FindField(message, "y");
  const auto* z_field = FindField(message, "z");
  const auto* ring_field = FindField(message, "ring");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr ||
      !FieldFitsPoint(*x_field, message.point_step) ||
      !FieldFitsPoint(*y_field, message.point_step) ||
      !FieldFitsPoint(*z_field, message.point_step) ||
      (ring_field != nullptr &&
       !FieldFitsPoint(*ring_field, message.point_step))) {
    if (error != nullptr) {
      *error = "PointCloud2 is missing a valid x/y/z field";
    }
    return false;
  }
  if (message.point_step == 0 ||
      message.row_step < message.point_step * message.width ||
      message.data.size() <
          static_cast<std::size_t>(message.row_step) * message.height) {
    if (error != nullptr) {
      *error = "PointCloud2 layout is invalid";
    }
    return false;
  }

  cloud->reserve(cloud->size() +
                 static_cast<std::size_t>(message.width) * message.height);
  for (std::uint32_t row = 0; row < message.height; ++row) {
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const std::size_t offset =
          static_cast<std::size_t>(row) * message.row_step +
          static_cast<std::size_t>(column) * message.point_step;
      const std::uint8_t* point = message.data.data() + offset;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!ReadNumber(point, *x_field, message.is_bigendian, &x) ||
          !ReadNumber(point, *y_field, message.is_bigendian, &y) ||
          !ReadNumber(point, *z_field, message.is_bigendian, &z)) {
        if (error != nullptr) {
          *error = "PointCloud2 x/y/z field datatype is unsupported";
        }
        return false;
      }
      PointXYZRing converted;
      converted.x = static_cast<float>(x);
      converted.y = static_cast<float>(y);
      converted.z = static_cast<float>(z);
      converted.ring = 0xFFFF;
      if (ring_field != nullptr) {
        double ring = 0.0;
        if (!ReadNumber(point, *ring_field, message.is_bigendian, &ring)) {
          if (error != nullptr) {
            *error = "PointCloud2 ring field datatype is unsupported";
          }
          return false;
        }
        converted.ring = static_cast<std::uint16_t>(ring);
      }
      cloud->push_back(converted);
    }
  }
  *lidar_type = ring_field == nullptr ? LidarType::kSolidState
                                      : LidarType::kMechanical;
  return true;
}

template <typename LivoxMessage>
void AppendLivox(const LivoxMessage& message,
                 pcl::PointCloud<PointXYZRing>::Ptr cloud) {
  const std::size_t point_count =
      std::min(static_cast<std::size_t>(message.point_num),
               message.points.size());
  cloud->reserve(cloud->size() + point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    PointXYZRing converted;
    converted.x = message.points[index].x;
    converted.y = message.points[index].y;
    converted.z = message.points[index].z;
    converted.ring =
        static_cast<std::uint16_t>(message.points[index].line);
    cloud->push_back(converted);
  }
}

}  // namespace

bool LoadInputData(const Params& params,
                   InputData* data,
                   std::string* error) {
  if (data == nullptr) {
    if (error != nullptr) {
      *error = "input data pointer is null";
    }
    return false;
  }
  data->image = cv::imread(params.image_path, cv::IMREAD_UNCHANGED);
  if (data->image.empty()) {
    if (error != nullptr) {
      *error = "loading image failed: " + params.image_path;
    }
    return false;
  }
  data->cloud->clear();
  data->lidar_type = LidarType::kUnknown;
  data->message_type.clear();
  data->message_count = 0;

  try {
    rosbag::Bag bag(params.bag_path, rosbag::bagmode::Read);
    const std::vector<std::string> topics{params.lidar_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (const rosbag::MessageInstance& instance : view) {
      const std::string type = instance.getDataType();
      if (type == "livox_ros_driver/CustomMsg") {
        const auto message =
            instance.instantiate<livox_ros_driver::CustomMsg>();
        if (message != nullptr) {
          AppendLivox(*message, data->cloud);
          data->lidar_type = LidarType::kSolidState;
          data->message_type = type;
          ++data->message_count;
        }
        continue;
      }
      if (type == "livox_ros_driver2/CustomMsg") {
        const auto message =
            instance.instantiate<livox_ros_driver2::CustomMsg>();
        if (message != nullptr) {
          AppendLivox(*message, data->cloud);
          data->lidar_type = LidarType::kSolidState;
          data->message_type = type;
          ++data->message_count;
        }
        continue;
      }
      if (type == "sensor_msgs/PointCloud2") {
        const auto message = instance.instantiate<sensor_msgs::PointCloud2>();
        if (message != nullptr) {
          if (!AppendPointCloud2(*message, data->cloud, &data->lidar_type,
                                 error)) {
            return false;
          }
          data->message_type = type;
          ++data->message_count;
        }
      }
    }
  } catch (const rosbag::BagException& exception) {
    if (error != nullptr) {
      *error = "loading rosbag failed: " + params.bag_path + ": " +
               exception.what();
    }
    return false;
  }

  if (data->message_count == 0 || data->cloud->empty() ||
      data->lidar_type == LidarType::kUnknown) {
    if (error != nullptr) {
      std::ostringstream message;
      message << "no supported LiDAR messages found on topic '"
              << params.lidar_topic << "' in " << params.bag_path;
      *error = message.str();
    }
    return false;
  }
  ROS_INFO_STREAM("Loaded " << data->cloud->size() << " points from "
                             << data->message_count << " "
                             << data->message_type << " message(s) on "
                             << params.lidar_topic);
  return true;
}

}  // namespace ros1
}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo
