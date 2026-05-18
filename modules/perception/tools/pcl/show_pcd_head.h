#pragma once

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>

inline void show_pcd_head(const std::string& file) {
  pcl::PCLPointCloud2 cloud;
  pcl::io::loadPCDFile(file, cloud);

  /* pcl > 1.12
  for (const auto& field : cloud.fields) {
    std::cout << "Field: " << field.name << " | Offset: " << field.offset
              << " | Datatype: " << field.datatype
              << " | Count: " << field.count << std::endl;
  }
  */
}