#ifndef MATH_UTILS_
#define MATH_UTILS_

#include <Eigen/Core>

#include "common_data.h"

using namespace CommonData;

namespace math_utils {

inline double RadToDeg(double rad) { return rad * (180.0 / M_PI); }

// 调整到 -π 到 π 的范围内
inline double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

/**
 * @brief 计算两个向量间的偏航角
 * @return 返回计算的偏航角 单位：角度
 */
inline double ComputeYaw(Eigen::Vector2d curr_direction,
                         Eigen::Vector2d base_direction) {
  double yaw = std::atan2(curr_direction[1], curr_direction[0]) -
               std::atan2(base_direction[1], base_direction[0]);

  yaw = NormalizeAngle(yaw);
  /*
   * 调整角度范围到 -PI 和 PI 之间,可以用下面的，std::remainder 函数返回余数
   * yaw = std::remainder(yaw + M_PI, 2.0 * M_PI) - M_PI;
   */
  yaw = RadToDeg(yaw);

  return yaw;
}

/**
 * @brief 计算两点间距离
 *
 * @param start 起始点
 * @param end  终点
 * @param meters_per_pix  栅格分辨率
 * @return double 两点间的距离（world坐标系下）
 */
inline double ComputeDistance(const Point2D& start, const Point2D& end,
                              double meters_per_pix) {
  double dist = std::sqrt(std::pow((end.x - start.x), 2) +
                          std::pow((end.y - start.y), 2));
  dist = dist * meters_per_pix;
  return dist;
}

/**
 * @brief 将给定的索引值转换为在指定列表长度范围内的“循环”索引
 * 当索引值超出列表长度时，它会在列表长度范围内循环，以防止越界
 * @param index 原始索引
 * @param list_length 列表长度
 * @return int 转换后的索引，将位于范围[0,list_length-1]内
 */
inline int WrappedIndex(int index, int list_length) {
  int wrapped_index = (index % list_length + list_length) % list_length;
  return wrapped_index;
}

}  // namespace math_utils
 
#endif  // !MATH_UTILS_