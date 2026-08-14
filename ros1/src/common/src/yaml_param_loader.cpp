#include "codexopen_ros1/yaml_param_loader.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <stdexcept>

#include <XmlRpcValue.h>
#include <ros/console.h>
#include <yaml-cpp/yaml.h>

namespace codexopen_ros1 {
namespace {

XmlRpc::XmlRpcValue ToXmlRpc(const YAML::Node& node) {
  if (node.IsSequence()) {
    XmlRpc::XmlRpcValue result;
    result.setSize(static_cast<int>(node.size()));
    for (std::size_t index = 0; index < node.size(); ++index) {
      result[static_cast<int>(index)] = ToXmlRpc(node[index]);
    }
    return result;
  }

  if (node.IsMap()) {
    XmlRpc::XmlRpcValue result;
    for (const auto& entry : node) {
      result[entry.first.as<std::string>()] = ToXmlRpc(entry.second);
    }
    return result;
  }

  if (!node.IsScalar()) {
    return XmlRpc::XmlRpcValue();
  }

  const std::string value = node.Scalar();
  std::string lowercase = value;
  std::transform(lowercase.begin(), lowercase.end(), lowercase.begin(),
                 [](unsigned char character) { return std::tolower(character); });
  if (lowercase == "true") {
    return XmlRpc::XmlRpcValue(true);
  }
  if (lowercase == "false") {
    return XmlRpc::XmlRpcValue(false);
  }

  char* end = nullptr;
  errno = 0;
  const long integer = std::strtol(value.c_str(), &end, 10);
  if (errno == 0 && end != value.c_str() && *end == '\0') {
    return XmlRpc::XmlRpcValue(static_cast<int>(integer));
  }

  errno = 0;
  end = nullptr;
  const double floating = std::strtod(value.c_str(), &end);
  if (errno == 0 && end != value.c_str() && *end == '\0') {
    return XmlRpc::XmlRpcValue(floating);
  }
  return XmlRpc::XmlRpcValue(value);
}

void SetParameters(const YAML::Node& values,
                   const std::string& prefix,
                   ros::NodeHandle& private_node) {
  for (const auto& entry : values) {
    const std::string key = entry.first.as<std::string>();
    const std::string name = prefix.empty() ? key : prefix + "/" + key;
    if (entry.second.IsMap()) {
      SetParameters(entry.second, name, private_node);
    } else {
      private_node.setParam(name, ToXmlRpc(entry.second));
    }
  }
}

}  // namespace

bool LoadYamlParameters(const std::string& path,
                        const std::string& section,
                        ros::NodeHandle& private_node) {
  try {
    YAML::Node values = YAML::LoadFile(path);
    if (!section.empty()) {
      values = values[section];
    }
    if (!values || !values.IsMap()) {
      ROS_ERROR_STREAM("Configuration section '" << section
                       << "' is missing or is not a map in " << path);
      return false;
    }
    SetParameters(values, "", private_node);
    ROS_INFO_STREAM("Loaded configuration " << path
                    << (section.empty() ? "" : " [" + section + "]"));
    return true;
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Cannot load configuration " << path << ": "
                     << error.what());
    return false;
  }
}

}  // namespace codexopen_ros1
