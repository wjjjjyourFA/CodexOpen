#ifndef COLORS_CV_H
#define COLORS_CV_H

#include <opencv2/opencv.hpp>

cv::Scalar GetColor(int index);

cv::Scalar GetColorByDistance(float distance);

void DrawText(cv::Mat& image, const cv::Point& anchor, const std::string& txt,
              const cv::Scalar& color, int offset_y = 0,
              double font_scale = 0.6, int thickness = 1);

#endif  // COLORS_CV_H
