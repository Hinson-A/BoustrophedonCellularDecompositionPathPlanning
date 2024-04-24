#ifndef COVER_PLANNING_H_
#define COVER_PLANNING_H_
#include "bcd.h"

namespace cover_planning {
//确定点在cell_graph中的索引
std::vector<int> DetermineCellIndex(std::vector<CellNode>& cell_graph,
                                    const Point2D& point);

std::vector<Point2D> ComputeCellCornerPoints(const CellNode& cell);

std::deque<Point2D> GetBoustrophedonPath(std::vector<CellNode>& cell_graph,
                                         CellNode cell, int corner_indicator,
                                         int robot_radius);

// 去除原始轨迹中连续重复的位置点，以减少数据量或提高轨迹的精确度
std::deque<Point2D> FilterTrajectory(
    const std::deque<std::deque<Point2D>>& raw_trajectory);

/**
 * @brief 检查给定路径的连续性和重复性
 * TODO：也只是打印下，并没有做其他特别处理了
 * @param path 指定路径
 */
void CheckPathConsistency(const std::deque<Point2D>& path);

// TODO perf: use p2p path
// cell内部起始点到终点的路径：先水平-在垂直路径
std::deque<Point2D> WalkInsideCell(CellNode cell, const Point2D& start,
                                   const Point2D& end);

/**
 * @brief
 * 从给定cell_graph中的指定cell开始，递归地遍历图中的cell，并构建cell访问顺序。
 *TODO 不适用，每个cell切割后，采用获取最邻近原则，或者TSP求解
 * @param cell_graph cell_graph的引用，包含所有cell的信息。
 * @param cell_index 要开始遍历的cell的索引。
 * @param unvisited_counter 未访问的cell数量的引用，将在遍历过程中更新。
 * @param path 保存遍历cell的deque容器。
 */
void WalkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index,
                      int& unvisited_counter, std::deque<CellNode>& path);

}  // namespace cover_planning
#endif  // !COVER_PLANNING_H_
