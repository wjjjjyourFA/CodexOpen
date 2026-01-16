#include "modules/tools/sensor_calibration/common/yaml_writer.h"

void YamlMatWriter::writeMat(const std::string& name, const cv::Mat& mat) {
  std::ofstream fout;
  if (first_write_) {
    fout.open(filename_, std::ios::out | std::ios::trunc);  // ✅ 第一次：覆盖
    fout << "%YAML:1.0\n---\n";
    first_write_ = false;
  } else {
    fout.open(filename_, std::ios::app);  // ✅ 后续：追加
  }

  if (!fout.is_open()) {
    std::cerr << "Cannot open " << filename_ << " for writing!" << std::endl;
    return;
  }

  fout << name << ": !!opencv-matrix\n";
  fout << "   rows: " << mat.rows << "\n";
  fout << "   cols: " << mat.cols << "\n";
  fout << "   dt: "
       << ((mat.depth() == CV_64F)   ? "d"
           : (mat.depth() == CV_32F) ? "f"
                                     : "i")
       << "\n";
  fout << "   data: [ ";

  fout << std::fixed;
  // fout << std::fixed << std::setprecision(10);

  int total = mat.total() * mat.channels();
  for (int i = 0; i < total; ++i) {
    double val = 0.0;
    if (mat.depth() == CV_64F) {
      val = mat.at<double>(i);
    } else if (mat.depth() == CV_32F) {
      val = mat.at<float>(i);
    } else {
      fout << static_cast<int>(mat.at<uchar>(i));
    }

    // 输出格式处理
    if (std::fabs(val - std::round(val)) < 1e-9) {
      // 整数输出成 960. 这样的形式
      fout << static_cast<long long>(std::llround(val)) << ".";
    } else {
      // 浮点数保留 8 位小数
      fout << std::setprecision(8) << val;
    }

    if (i != total - 1) fout << ", ";
  }
  fout << " ]\n";
}