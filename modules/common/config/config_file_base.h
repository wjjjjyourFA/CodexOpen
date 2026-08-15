#ifndef CONFIG_FILE_BASE_H
#define CONFIG_FILE_BASE_H

#pragma once

#include <fstream>  // 用于 std::ifstream
#include <iostream>  // 用于 std::cout
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace jojo {
namespace common {
namespace config {

// 做 配置解析，而不是 运行时参数管理
class ConfigFileBase {
 public:
  ConfigFileBase()          = default;
  virtual ~ConfigFileBase() = default;

  virtual void LoadConfig(const std::string& config_path) = 0;

  void set_name(const std::string& name) { this->name_ = name; }

  const std::string& get_name() const { return this->name_; }

 protected:
  std::string name_ = "";

  // 配置文件里的字符串数组
  std::vector<std::string> ReadStringArray(
      const boost::property_tree::ptree& pt, const std::string& prefix,
      int count) const;

  // 解析行为
  std::vector<std::string> ParseCommaSeparated(
      const boost::property_tree::ptree& pt, const std::string& key) const;
};

}  // namespace config
}  // namespace common
}  // namespace jojo

#endif  // CONFIG_FILE_BASE_H
