// visualization.h

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <deque>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "common_data.h"

using namespace CommonData;

// 定义颜色板中的颜色数量
const int palette_colors = 1530;

// PATH_MODE路径可视化:机器人从起点到终点的完整路径;
// ROBOT_MODE:运动可视化: 模拟机器人的实际运动
enum VisualizationMode { PATH_MODE, ROBOT_MODE };

namespace map_visualization {

//可视化轮廓: 在地图上绘制提取的轮廓，并逐个显示轮廓
void VisualizeExtractedContours(
    const cv::Mat& map, const std::vector<std::vector<cv::Point>>& contours);

// 可视化轨迹
void VisualizeTrajectory(const cv::Mat& original_map,
                         const std::deque<Point2D>& path, int robot_radius,
                         int vis_mode, int time_interval = 10,
                         int colors = palette_colors);
// 初始化颜色映射
void InitializeColorMap(std::deque<cv::Scalar>& JetColorMap, int repeat_times);

// 更新颜色映射
void UpdateColorMap(std::deque<cv::Scalar>& color_map);

void DrawCells(cv::Mat& map, const CellNode& cell,
               cv::Scalar color = cv::Scalar(100, 100, 100));

}  // namespace map_visualization
#endif  // VISUALIZATION_H_