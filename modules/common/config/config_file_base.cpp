#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace common {
namespace config {

std::vector<std::string> ConfigFileBase::ReadStringArray(
    const boost::property_tree::ptree& pt, const std::string& prefix,
    int count) const {
  std::vector<std::string> values;
  for (int i = 1; i <= count; ++i) {
    std::string key = prefix + std::to_string(i);
    std::string val = pt.get<std::string>(key, "");  // 默认值 ""
    if (!val.empty()) {
      values.push_back(val);
    }
  }
  return values;
}

std::vector<std::string> ConfigFileBase::ParseCommaSeparated(
    const boost::property_tree::ptree& pt, const std::string& key) const {
  std::vector<std::string> items;
  const std::string line = pt.get<std::string>(key, "");

  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    // 去掉前后空格
    item.erase(item.begin(),
               std::find_if(item.begin(), item.end(), [](unsigned char ch) {
                 return !std::isspace(ch);
               }));
    item.erase(std::find_if(item.rbegin(), item.rend(),
                            [](unsigned char ch) { return !std::isspace(ch); })
                   .base(),
               item.end());

    if (!item.empty()) {
      items.push_back(item);
    }
  }
  return items;
}

}  // namespace config
}  // namespace common
}  // namespace jojo
