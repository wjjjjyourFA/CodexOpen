#include <iostream>

#include <opencv2/opencv.hpp>

class YamlMatWriter {
 public:
  YamlMatWriter(const std::string& filename)
      : filename_(filename), first_write_(true) {}

  void writeMat(const std::string& name, const cv::Mat& mat);

 private:
  std::string filename_;
  bool first_write_;
};