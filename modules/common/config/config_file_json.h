#ifndef CONFIG_FILE_JSON_H
#define CONFIG_FILE_JSON_H

#pragma once

#include <nlohmann/json.hpp>

namespace jojo {
namespace common {
namespace config {
using json = nlohmann::json;

class ConfigFileJson {
 public:
  ConfigFileJson()          = default;
  virtual ~ConfigFileJson() = default;

  virtual void LoadConfig(const std::string& config_path) = 0;

  void set_name(const std::string& name) { this->name_ = name; }

  const std::string& get_name() { return this->name_; }

 protected:
  std::string name_ = "";
};

// ========= 通用安全读取 =========
template <typename T>
T GetOrDefault(const json& j, const std::string& key, const T& def) {
  if (j.contains(key) && !j.at(key).is_null()) {
    return j.at(key).get<T>();
  }
  return def;
}

}  // namespace config
}  // namespace common
}  // namespace jojo

#endif  // CONFIG_FILE_JSON_H