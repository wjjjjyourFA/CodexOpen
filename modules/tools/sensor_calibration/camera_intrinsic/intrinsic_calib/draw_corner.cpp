/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 * @Author: HuangWei
 * @Date: 2023-06-08 17:48:28
 * @LastEditors: HuangWei
 * @LastEditTime: 2023-06-08 17:58:27
 * @FilePath: /camera_intrinsic/intrinsic_calib/src/drawCorner_jojo.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by JOJO, All Rights Reserved. 
 */
#include <opencv2/opencv.hpp>

void drawConrner(cv::Mat image_color) {
  cv::Mat image_gray;
  cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point2f> corners;

  // 标定板 横向多少角点，纵向多少角点
  cv::Size corner_size = cv::Size(3, 7);

  bool ret = cv::findChessboardCorners(
      image_gray, corner_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

  // 指定亚像素计算迭代标注
  cv::TermCriteria criteria = cv::TermCriteria(
      cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.1);

  // 亚像素检测
  cv::cornerSubPix(image_gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                   criteria);

  // 角点绘制
  cv::drawChessboardCorners(image_color, cv::Size(3, 7), corners, ret);

  cv::imshow("chessboard corners", image_color);
  cv::waitKey(0);
};
