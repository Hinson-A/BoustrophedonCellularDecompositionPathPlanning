#pragma once

#include <deque>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "common_data.h"
using namespace CommonData;
typedef std::vector<Point2D> Polygon;
typedef std::vector<Polygon> PolygonList;
/** 地图默认是空闲区域为白色，障碍物为黑色 **/

// PATH_MODE路径可视化:机器人从起点到终点的完整路径;
// ROBOT_MODE:运动可视化: 模拟机器人的实际运动
enum VisualizationMode { PATH_MODE, ROBOT_MODE };

//矩形顶点常量定义
const int TOP_LEFT = 0;
const int BOTTOM_LEFT = 1;
const int BOTTOM_RIGHT = 2;
const int TOP_RIGHT = 3;

const int palette_colors = 1530;

class BoustrophedonCellDecomposition {
 public:
  BoustrophedonCellDecomposition() {}

  /**
   * @brief 根据地图和障碍物信息构建单元格图
   *
   * 该函数接收原始地图以及墙和障碍物的轮廓信息，然后构建一个单元格图。
   * 单元格图是一种数据结构，用于表示地图上的可行驶区域和障碍物区域，是路径规划算法的基础。
   * 此函数首先生成事件列表，然后通过扫描线算法将事件列表转换为单元格图。
   *
   * @param original_map
   * 原始地图的OpenCV矩阵表示，其中白色代表空闲区域，黑色代表障碍物
   * @param wall_contours 墙的轮廓点，以多边形顶点列表的形式给出
   * @param obstacle_contours
   * 所有障碍物的轮廓点，每个障碍物的轮廓点以多边形顶点列表的形式给出
   * @param wall 表示地图外部边界的多边形
   * @param obstacles 表示地图上所有障碍物的多边形列表
   * @return 一个CellNode对象的向量，每个CellNode代表单元格图中的一个单元格
   */
  std::vector<CellNode> ConstructCellGraph(
      const cv::Mat& original_map,
      const std::vector<std::vector<cv::Point>>& wall_contours,
      const std::vector<std::vector<cv::Point>>& obstacle_contours,
      const Polygon& wall, const PolygonList& obstacles);
  void CheckSlicelist(const std::deque<std::deque<Event>>& slice_list);

  /**
   * @brief 为地图中的障碍物生成事件列表
   *
   * 该函数接收一张地图和障碍物的多边形表示，然后为每个障碍物的每个顶点
   * 创建一个Event对象，并初始化其位置和类型。这些事件随后可用于路径规划算法。
   *
   * @param map
   * 表示地图的OpenCV矩阵，其中障碍物被标记为黑色，空闲区域被标记为白色
   * @param polygons 包含所有障碍物多边形顶点的列表
   * @return 一个Event对象的向量，每个对象代表障碍物上的一个关键点
   */
  std::vector<Event> GenerateObstacleEventList(const cv::Mat& map,
                                               const PolygonList& polygons);

  /**
   * @brief 根据地图和外部轮廓生成墙事件列表
   *
   * 此函数遍历地图中的墙轮廓，并生成一系列的事件对象，这些事件对象
   * 将用于后续的路径规划算法。事件对象包含了墙轮廓的关键点，例如
   * 墙的起始点和终止点，以及可能的转角点。
   *
   * @param map 表示地图的OpenCV矩阵，其中墙被标记为黑色，空闲区域被标记为白色
   * @param external_contour 表示地图外部边界的多边形轮廓
   * @return 一个事件列表，包含了墙轮廓的关键事件点
   */
  std::vector<Event> GenerateWallEventList(const cv::Mat& map,
                                           const Polygon& external_contour);
  /**
   * @brief 根据墙事件列表和障碍物事件列表生成切片列表
   * @param wall_event_list 墙事件列表，包含了墙的顶点事件
   * @param obstacle_event_list 障碍物事件列表，包含了障碍物的顶点事件
   * @return 一个由事件切片组成的双端队列，每个切片是一个事件队列，按x坐标排序
   */
  std::deque<std::deque<Event>> SliceListGenerator(
      const std::vector<Event>& wall_event_list,
      const std::vector<Event>& obstacle_event_list);

  void ExecuteCellDecomposition(
      std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice,
      std::vector<int>& original_cell_index_slice,
      const std::deque<std::deque<Event>>& slice_list);

 private:
  void AllocateWallEventType(const cv::Mat& map,
                             std::vector<Event>& event_list);
  std::vector<Event> InitializeEventList(const Polygon& polygon,
                                         int polygon_index);
  int CountCells(const std::deque<Event>& slice, int curr_idx);
  std::deque<Event> FilterSlice(const std::deque<Event>& slice);

 private:
  /**
   * @brief 为障碍物事件列表中的每个事件分配具体的事件类型
   *
   * 该函数分析地图和事件列表，为每个事件对象分配一个详细的事件类型，
   * 这些类型包括IN, OUT, MIDDLE, CEILING,
   * FLOOR等，这些类型基于事件点与障碍物的相对位置。
   *
   * @param map 表示地图的OpenCV矩阵
   * @param event_list 需要为其成员分配事件类型的事件列表
   */
  void AllocateObstacleEventType(const cv::Mat& map,
                                 std::vector<Event>& event_list);

  /**
   * @brief 在单元格图中执行开放操作，用于创建新的单元格或修改现有单元格的边界
   * @param cell_graph 单元格图，包含了所有单元格的信息
   * @param curr_cell_idx 当前处理的单元格索引
   * @param in 进入点，表示从哪里进入当前单元格
   * @param c 切片中的一个点，通常用于确定天花板或地板的边界
   * @param f 切片中的另一个点，与c点配合使用
   * @param rewrite 是否重写当前单元格的边界，如果为false，则创建新的单元格
   */
  void ExecuteOpenOperation(std::vector<CellNode>& cell_graph,
                            int curr_cell_idx, Point2D in, Point2D c, Point2D f,
                            bool rewrite = false);

  /**
   * @brief 在单元格图中执行关闭操作，用于完成单元格的边界
   * @param cell_graph 单元格图，包含了所有单元格的信息
   * @param top_cell_idx 要关闭的单元格的顶部单元格索引
   * @param bottom_cell_idx 要关闭的单元格的底部单元格索引
   * @param c 切片中的一个点，用于确定新单元格的天花板或地板的边界
   * @param f 切片中的另一个点，与c点配合使用
   * @param rewrite
   * 是否重写涉及的单元格的边界，如果为false，则创建一个新的单元格来连接top_cell和bottom_cell
   */
  void ExecuteCloseOperation(std::vector<CellNode>& cell_graph,
                             int top_cell_idx, int bottom_cell_idx, Point2D c,
                             Point2D f, bool rewrite = false);
  void ExecuteCeilOperation(std::vector<CellNode>& cell_graph,
                            int curr_cell_idx, const Point2D& ceil_point);
  void ExecuteFloorOperation(std::vector<CellNode>& cell_graph,
                             int curr_cell_idx, const Point2D& floor_point);
  void ExecuteOpenOperation(std::vector<CellNode>& cell_graph,
                            int curr_cell_idx, Point2D in_top,
                            Point2D in_bottom, Point2D c, Point2D f,
                            bool rewrite = false);
  void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph,
                                 Point2D inner_in);
  void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph,
                                 Point2D inner_in_top, Point2D inner_in_bottom);
  void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph,
                                  int curr_cell_idx, Point2D inner_out);

  void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph,
                                  int curr_cell_idx, Point2D inner_out_top,
                                  Point2D inner_out_bottom);
};
