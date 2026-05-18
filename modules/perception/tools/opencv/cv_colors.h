#ifndef COLORS_CV_H
#define COLORS_CV_H

#include <opencv2/opencv.hpp>

cv::Scalar GetColor(int index);

cv::Scalar GetColorByDistance(float distance, float max_range = 100.0f,
                              float min_range = 0.5f);

cv::Scalar GetColorByNorm(float t);

cv::Scalar GetColorByHeight(float height, float min_h, float max_h);

cv::Scalar HSV2RGB(cv::Scalar scalar);

void DrawText(cv::Mat& image, const cv::Point& anchor, const std::string& txt,
              const cv::Scalar& color, int offset_y = 0,
              double font_scale = 0.6, int thickness = 1);

#endif  // COLORS_CV_H
