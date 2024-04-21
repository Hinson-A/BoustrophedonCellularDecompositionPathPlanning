#include "map_sdk.h"

namespace map_sdk {

//从原始地图中提取墙和障碍物的轮廓，并根据机器人的半径对轮廓进行处理
void ExtractContours(const cv::Mat& original_map,
                     std::vector<std::vector<cv::Point>>& wall_contours,
                     std::vector<std::vector<cv::Point>>& obstacle_contours,
                     int robot_radius) {
  ExtractRawContours(original_map, wall_contours, obstacle_contours);

  // 如果半径不为0，就以原边界点为圆心，机器人半径大小外扩
  if (robot_radius != 0) {
    cv::Mat3b canvas = cv::Mat3b(original_map.size(), CV_8U);
    canvas.setTo(cv::Scalar(255, 255, 255));

    cv::fillPoly(canvas, wall_contours, cv::Scalar(0, 0, 0));
    for (const auto& point : wall_contours.front()) {
      cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
    }

    cv::fillPoly(canvas, obstacle_contours, cv::Scalar(255, 255, 255));
    for (const auto& obstacle_contour : obstacle_contours) {
      for (const auto& point : obstacle_contour) {
        cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
      }
    }

    cv::Mat canvas_;
    cv::cvtColor(canvas, canvas_, cv::COLOR_BGR2GRAY);
    cv::threshold(canvas_, canvas_, 200, 255, cv::THRESH_BINARY_INV);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(robot_radius, robot_radius),
        cv::Point(-1, -1));
    cv::morphologyEx(canvas_, canvas_, cv::MORPH_OPEN, kernel);

    ExtractRawContours(canvas_, wall_contours, obstacle_contours);

    std::vector<cv::Point> processed_wall_contour;
    // 对图像轮廓点进行多边形拟合
    cv::approxPolyDP(cv::Mat(wall_contours.front()), processed_wall_contour, 1,
                     true);

    std::vector<std::vector<cv::Point>> processed_obstacle_contours(
        obstacle_contours.size());
    for (int i = 0; i < obstacle_contours.size(); i++) {
      cv::approxPolyDP(cv::Mat(obstacle_contours[i]),
                       processed_obstacle_contours[i], 1, true);
    }

    wall_contours = {processed_wall_contour};
    obstacle_contours = processed_obstacle_contours;
  }
}

//从原始地图图像中提取墙和障碍物的原始轮廓
void ExtractRawContours(
    const cv::Mat& original_map,
    std::vector<std::vector<cv::Point>>& raw_wall_contours,
    std::vector<std::vector<cv::Point>>& raw_obstacle_contours) {
  cv::Mat map = original_map.clone();
  //  图像二值化：128-255
  //  的置为0，否则为1,即将非黑色（即值为0）的像素点设置为255（白色）
  cv::threshold(map, map, 128, 255, cv::THRESH_BINARY_INV);
  //  从灰度空间转换到BGR颜色空间
  cv::cvtColor(map, map, cv::COLOR_GRAY2BGR);

  //  提取图像轮廓，cv::RETR_EXTERNAL标志来只检索最外层的轮廓，且这些轮廓不与其他轮廓连接
  std::vector<std::vector<cv::Point>> contours;  //所有检测到的轮廓
  cv::findContours(original_map.clone(), contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);

  /* 索引排序:
   *使用 std::iota 填充wall_cnt_indices向量，使其包含从0开始的连续整数
   *使用 std::sort 和lambda表达式对轮廓索引进行排序，根据轮廓面积从大到小排列;
   *cv::contourArea函数用于计算轮廓的面积
   */
  std::vector<int> wall_cnt_indices(contours.size());
  // iota将一个从value递增的数列给[first, last)的容器赋值
  //将每个轮廓都赋值一个唯一的索引与之对应
  std::iota(wall_cnt_indices.begin(), wall_cnt_indices.end(), 0);

  // lambda表达式
  /*cv::contourArea计算轮廓面积, 对所有轮廓按照面积大小从大到小排序*/
  std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(),
            [&contours](int lhs, int rhs) {
              return cv::contourArea(contours[lhs]) >
                     cv::contourArea(contours[rhs]);
            });

  //选择面积最大的轮廓作为墙的轮廓
  std::vector<cv::Point> raw_wall_contour = contours[wall_cnt_indices.front()];
  raw_wall_contours = {raw_wall_contour};

  // 创建一个与原始地图大小相同的地图，并初始化为全白色
  cv::Mat mask = cv::Mat(original_map.size(), original_map.type(), 255);
  cv::fillPoly(mask, raw_wall_contours, 0);  // 以最大的轮廓为边，填充为黑色
  //   cv::imshow("mask", mask);
  cv::Mat base = original_map.clone();
  // cv::imshow("origin_map_base", base);

  // 将mask与原始地图相加,墙区域将被设置为白色，障碍物区域保持黑色。
  base += mask;
  // cv::imshow("base+mask", base);

  //颜色反转:使得墙区域变为黑色，障碍物区域保持白色。
  cv::threshold(base, base, 128, 255, cv::THRESH_BINARY_INV);
  // cv::imshow("after_base", base);

  //重新提取轮廓: 这次提取的是障碍物的轮廓
  cv::findContours(base, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  raw_obstacle_contours = contours;
}

}  // namespace map_sdk