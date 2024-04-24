#ifndef MAP_SDK_H_
#define MAP_SDK_H_

#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "common_data.h"
using namespace CommonData;

namespace map_sdk {

// 读取地图文件并返回一个灰度图像
cv::Mat1b ReadMap(const std::string& map_file_path);

// 对读取的地图进行预处理，将其阈值化为二进制图像
cv::Mat1b PreprocessMap(const cv::Mat1b& original_map);
/**
 * @brief 从原始地图中提取墙和障碍物的轮廓，并根据机器人的半径对轮廓进行处理
 *
 * @param original_map 原始的地图图像
 * @param wall_contours 用于存储墙的轮廓点
 * @param obstacle_contours 用于存储障碍物的轮廓点
 * @param robot_radius (默认值为0):
 * 机器人的半径，用于确定是否需要对轮廓进行外扩处理
 */
void ExtractContours(const cv::Mat& original_map,
                     std::vector<std::vector<cv::Point>>& wall_contours,
                     std::vector<std::vector<cv::Point>>& obstacle_contours,
                     int robot_radius = 0);

/**
 * @brief 从原始地图图像中提取墙和障碍物的原始轮廓
 *
 * @param original_map 原始的地图图像
 * @param raw_wall_contours 用于存储墙的原始轮廓点
 * @param raw_obstacle_contours 于存储所有障碍物的原始轮廓点
 */
void ExtractRawContours(
    const cv::Mat& original_map,
    std::vector<std::vector<cv::Point>>& raw_wall_contours,
    std::vector<std::vector<cv::Point>>& raw_obstacle_contours);

PolygonList ConstructObstacles(
    const cv::Mat& original_map,
    const std::vector<std::vector<cv::Point>>& obstacle_contours);

Polygon ConstructWall(const cv::Mat& original_map,
                      std::vector<cv::Point>& wall_contour);

// 构造一个默认的墙多边形数据，该墙体多边形围绕地图的边界
Polygon ConstructDefaultWall(const cv::Mat& original_map);

}  // namespace map_sdk

#endif  // !MAP_SDK_H_