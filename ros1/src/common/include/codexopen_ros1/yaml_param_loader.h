#ifndef CODEXOPEN_ROS1_YAML_PARAM_LOADER_H
#define CODEXOPEN_ROS1_YAML_PARAM_LOADER_H

#include <string>

#include <ros/node_handle.h>

namespace codexopen_ros1 {

// Load a YAML map (or one named section of it) into the supplied ROS
// namespace. Nested maps are written leaf-by-leaf so a later interface file
// can override transport fields without replacing algorithm parameters.
bool LoadYamlParameters(const std::string& path,
                        const std::string& section,
                        ros::NodeHandle& private_node);

}  // namespace codexopen_ros1

#endif
