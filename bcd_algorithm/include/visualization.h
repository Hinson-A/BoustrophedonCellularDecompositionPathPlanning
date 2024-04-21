// visualization.h

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <opencv2/core/core.hpp>
#include <deque>

// 定义颜色板中的颜色数量
const int palette_colors = 1530;

// 初始化颜色映射
void InitializeColorMap(std::deque<cv::Scalar>& color_map);

// 更新颜色映射
void UpdateColorMap(std::deque<cv::Scalar>& color_map);

// 可视化轨迹
void VisualizeTrajectory(const cv::Mat& map, const std::deque<Point2D>& path, int robot_radius, VisualizationMode vis_mode, int time_interval = 10);

#endif  // VISUALIZATION_H_