#include "cover_planning.h"

namespace cover_planning {

//确定点在cell_graph中的索引
std::vector<int> DetermineCellIndex(std::vector<CellNode>& cell_graph,
                                    const Point2D& point) {
  std::vector<int> cell_index;

  for (int i = 0; i < cell_graph.size(); i++) {
    for (int j = 0; j < cell_graph[i].ceiling.size(); j++) {
      if (point.x == cell_graph[i].ceiling[j].x &&
          point.y >= cell_graph[i].ceiling[j].y &&
          point.y <= cell_graph[i].floor[j].y) {
        cell_index.emplace_back(int(i));
      }
    }
  }
  return cell_index;
}

std::vector<Point2D> ComputeCellCornerPoints(const CellNode& cell) {
  Point2D topleft = cell.ceiling.front();
  Point2D bottomleft = cell.floor.front();
  Point2D bottomright = cell.floor.back();
  Point2D topright = cell.ceiling.back();

  // 按照TOPLEFT、BOTTOMLEFT、BOTTOMRIGHT、TOPRIGHT的顺序储存corner
  // points（逆时针）
  std::vector<Point2D> corner_points = {topleft, bottomleft, bottomright,
                                        topright};

  return corner_points;
}

std::deque<Point2D> GetBoustrophedonPath(std::vector<CellNode>& cell_graph,
                                         CellNode cell, int corner_indicator,
                                         int robot_radius) {
  int delta, increment;

  std::deque<Point2D> path;

  std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell);

  std::vector<Point2D> ceiling, floor;
  ceiling.assign(cell.ceiling.begin(), cell.ceiling.end());
  floor.assign(cell.floor.begin(), cell.floor.end());

  if (cell_graph[cell.cellIndex].isCleaned) {
    if (corner_indicator == TOP_LEFT) {
      path.emplace_back(corner_points[TOP_LEFT]);
    }
    if (corner_indicator == TOP_RIGHT) {
      path.emplace_back(corner_points[TOP_RIGHT]);
    }
    if (corner_indicator == BOTTOM_LEFT) {
      path.emplace_back(corner_points[BOTTOM_LEFT]);
    }
    if (corner_indicator == BOTTOM_RIGHT) {
      path.emplace_back(corner_points[BOTTOM_RIGHT]);
    }
  } else {
    if (corner_indicator == TOP_LEFT) {
      int x, y, y_start, y_end;
      bool reverse = false;

      for (int i = 0; i < ceiling.size(); i = i + (robot_radius + 1)) {
        x = ceiling[i].x;

        if (!reverse) {
          y_start = ceiling[i].y;
          y_end = floor[i].y;

          for (y = y_start; y <= y_end; y++) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(floor[i + 1].y - floor[i].y) >= 2) &&
              (i + 1 < floor.size())) {
            delta = floor[i + 1].y - floor[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(floor[i].x, floor[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着floor从左往右
              if (x + j >= floor.back().x) {
                i = i - (robot_radius - (j - 1));
                break;
              }

              // 提前转
              else if ((floor[i + (j)].y - floor[i + (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                delta = floor[i + (j + 1)].y - floor[i + (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      floor[i + (j)].x, floor[i + (j)].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((floor[i + (j + 1)].y - floor[i + (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                path.emplace_back(Point2D(floor[i + (j)].x, floor[i + (j)].y));

                delta = floor[i + (j + 1)].y - floor[i + (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(floor[i + (j + 1)].x,
                                            cell.floor[i + (j + 1)].y -
                                                abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(floor[i + (j)]);
              }
            }
          }

          reverse = !reverse;
        } else {
          y_start = floor[i].y;
          y_end = ceiling[i].y;

          for (y = y_start; y >= y_end; y--) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(ceiling[i + 1].y - ceiling[i].y) >= 2) &&
              (i + 1 < ceiling.size())) {
            delta = ceiling[i + 1].y - ceiling[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着ceiling从左往右
              if (x + j >= ceiling.back().x) {
                i = i - (robot_radius - (j - 1));
                break;
              }

              // 提前转
              else if ((ceiling[i + (j + 1)].y - ceiling[i + (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i + j].x, ceiling[i + j].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((ceiling[i + (j)].y - ceiling[i + (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                path.emplace_back(ceiling[i + (j)]);

                delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i + (j + 1)].x,
                      ceiling[i + (j + 1)].y + abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(ceiling[i + j]);
              }
            }
          }

          reverse = !reverse;
        }
      }
    }

    if (corner_indicator == TOP_RIGHT) {
      int x = 0, y = 0, y_start = 0, y_end = 0;
      bool reverse = false;

      for (int i = ceiling.size() - 1; i >= 0; i = i - (robot_radius + 1)) {
        x = ceiling[i].x;

        if (!reverse) {
          y_start = ceiling[i].y;
          y_end = floor[i].y;

          for (y = y_start; y <= y_end; y++) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(floor[i - 1].y - floor[i].y) >= 2) && (i - 1 >= 0)) {
            delta = floor[i - 1].y - floor[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(floor[i].x, floor[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着floor从右往左
              if (x - j <= floor.front().x) {
                i = i + (robot_radius - (j - 1));
                break;
              }
              // 提前转
              else if ((floor[i - (j)].y - floor[i - (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                delta = floor[i - (j + 1)].y - floor[i - (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      floor[i - (j)].x, floor[i - (j)].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((floor[i - (j + 1)].y - floor[i - (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                path.emplace_back(Point2D(floor[i - (j)].x, floor[i - (j)].y));

                delta = floor[i - (j + 1)].y - floor[i - (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(floor[i - (j + 1)].x,
                                            cell.floor[i - (j + 1)].y -
                                                abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(floor[i - (j)]);
              }
            }
          }

          reverse = !reverse;
        } else {
          y_start = floor[i].y;
          y_end = ceiling[i].y;

          for (y = y_start; y >= y_end; y--) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(ceiling[i - 1].y - ceiling[i].y) >= 2) &&
              (i - 1 >= 0)) {
            delta = ceiling[i - 1].y - ceiling[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着ceiling从右往左
              if (x - j <= ceiling.front().x) {
                i = i + (robot_radius - (j - 1));
                break;
              }
              // 提前转
              else if ((ceiling[i - (j + 1)].y - ceiling[i - (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i - j].x, ceiling[i - j].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((ceiling[i - (j)].y - ceiling[i - (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                path.emplace_back(ceiling[i - (j)]);

                delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i - (j + 1)].x,
                      ceiling[i - (j + 1)].y + abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(ceiling[i - j]);
              }
            }
          }

          reverse = !reverse;
        }
      }
    }

    if (corner_indicator == BOTTOM_LEFT) {
      int x = 0, y = 0, y_start = 0, y_end = 0;
      bool reverse = false;

      for (int i = 0; i < ceiling.size(); i = i + (robot_radius + 1)) {
        x = ceiling[i].x;

        if (!reverse) {
          y_start = floor[i].y;
          y_end = ceiling[i].y;

          for (y = y_start; y >= y_end; y--) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(ceiling[i + 1].y - ceiling[i].y) >= 2) &&
              (i + 1 < ceiling.size())) {
            delta = ceiling[i + 1].y - ceiling[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着ceiling从左往右
              if (x + j >= ceiling.back().x) {
                i = i - (robot_radius - (j - 1));
                break;
              }
              // 提前转
              else if ((ceiling[i + (j + 1)].y - ceiling[i + (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i + j].x, ceiling[i + j].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((ceiling[i + (j)].y - ceiling[i + (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                path.emplace_back(ceiling[i + (j)]);

                delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i + (j + 1)].x,
                      ceiling[i + (j + 1)].y + abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(ceiling[i + j]);
              }
            }
          }

          reverse = !reverse;
        } else {
          y_start = ceiling[i].y;
          y_end = floor[i].y;

          for (y = y_start; y <= y_end; y++) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(floor[i + 1].y - floor[i].y) >= 2) &&
              (i + 1 < floor.size())) {
            delta = floor[i + 1].y - floor[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(floor[i].x, floor[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着floor从左往右
              if (x + j >= floor.back().x) {
                i = i - (robot_radius - (j - 1));
                break;
              }

              // 提前转
              else if ((floor[i + (j)].y - floor[i + (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                delta = floor[i + (j + 1)].y - floor[i + (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      floor[i + (j)].x, floor[i + (j)].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((floor[i + (j + 1)].y - floor[i + (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                path.emplace_back(Point2D(floor[i + (j)].x, floor[i + (j)].y));

                delta = floor[i + (j + 1)].y - floor[i + (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(floor[i + (j + 1)].x,
                                            cell.floor[i + (j + 1)].y -
                                                abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(floor[i + (j)]);
              }
            }
          }

          reverse = !reverse;
        }
      }
    }

    if (corner_indicator == BOTTOM_RIGHT) {
      int x = 0, y = 0, y_start = 0, y_end = 0;
      bool reverse = false;

      for (int i = ceiling.size() - 1; i >= 0; i = i - (robot_radius + 1)) {
        x = ceiling[i].x;

        if (!reverse) {
          y_start = floor[i].y;
          y_end = ceiling[i].y;

          for (y = y_start; y >= y_end; y--) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(ceiling[i - 1].y - ceiling[i].y) >= 2) &&
              (i - 1 >= 0)) {
            delta = ceiling[i - 1].y - ceiling[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着ceiling从右往左
              if (x - j <= ceiling.front().x) {
                i = i + (robot_radius - (j - 1));
                break;
              }
              // 提前转
              else if ((ceiling[i - (j + 1)].y - ceiling[i - (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i - j].x, ceiling[i - j].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((ceiling[i - (j)].y - ceiling[i - (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                path.emplace_back(ceiling[i - (j)]);

                delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      ceiling[i - (j + 1)].x,
                      ceiling[i - (j + 1)].y + abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(ceiling[i - j]);
              }
            }
          }

          reverse = !reverse;
        } else {
          y_start = ceiling[i].y;
          y_end = floor[i].y;

          for (y = y_start; y <= y_end; y++) {
            path.emplace_back(Point2D(x, y));
          }

          if ((std::abs(floor[i - 1].y - floor[i].y) >= 2) && (i - 1 >= 0)) {
            delta = floor[i - 1].y - floor[i].y;
            increment = delta / abs(delta);
            for (int k = 1; k <= abs(delta); k++) {
              path.emplace_back(
                  Point2D(floor[i].x, floor[i].y + increment * (k)));
            }
          }

          if (robot_radius != 0) {
            for (int j = 1; j <= robot_radius + 1; j++) {
              // 沿着floor从右往左
              if (x - j <= floor.front().x) {
                i = i + (robot_radius - (j - 1));
                break;
              }
              // 提前转
              else if ((floor[i - (j)].y - floor[i - (j + 1)].y >= 2) &&
                       (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1)) {
                delta = floor[i - (j + 1)].y - floor[i - (j)].y;
                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(
                      floor[i - (j)].x, floor[i - (j)].y + increment * (k)));
                }
              }
              // 滞后转
              else if ((floor[i - (j + 1)].y - floor[i - (j)].y >= 2) &&
                       (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1)) {
                path.emplace_back(Point2D(floor[i - (j)].x, floor[i - (j)].y));

                delta = floor[i - (j + 1)].y - floor[i - (j)].y;

                increment = delta / abs(delta);
                for (int k = 0; k <= abs(delta); k++) {
                  path.emplace_back(Point2D(floor[i - (j + 1)].x,
                                            cell.floor[i - (j + 1)].y -
                                                abs(delta) + increment * (k)));
                }
              } else {
                path.emplace_back(floor[i - (j)]);
              }
            }
          }

          reverse = !reverse;
        }
      }
    }
  }

  return path;
}

// 去除原始轨迹中连续重复的位置点，以减少数据量或提高轨迹的精确度
std::deque<Point2D> FilterTrajectory(
    const std::deque<std::deque<Point2D>>& raw_trajectory) {
  std::deque<Point2D> trajectory;

  for (const auto& sub_trajectory : raw_trajectory) {
    for (const auto& position : sub_trajectory) {
      if (!trajectory.empty()) {
        if (position != trajectory.back()) {
          trajectory.emplace_back(position);
        }
      } else {
        trajectory.emplace_back(position);
      }
    }
  }

  return trajectory;
}

//检查给定路径的连续性和重复性
void CheckPathConsistency(const std::deque<Point2D>& path) {
  int breakpoints = 0;
  int duplicates = 0;
  // 它遍历路径中的每个点，检查相邻点之间的距离是否大于1
  // 并统计断点的数量；同时还统计路径中重复点的数量。
  for (int i = 1; i < path.size(); i++) {
    if (std::abs(path[WrappedIndex((i - 1), path.size())].x - path[i].x) > 1 ||
        std::abs(path[WrappedIndex((i - 1), path.size())].y - path[i].y) > 1) {
      breakpoints++;
      std::cout << "break points :"
                << path[WrappedIndex((i - 1), path.size())].x << ", "
                << path[WrappedIndex((i - 1), path.size())].y << "---->"
                << path[i].x << ", " << path[i].y << std::endl;
    }
    if (path[WrappedIndex((i - 1), path.size())] == path[i]) {
      duplicates++;
    }
  }
  std::cout << "breakpoints: " << breakpoints << std::endl;
  std::cout << "duplicates: " << duplicates << std::endl;
}

// TODO perf: use p2p path
// cell内部起始点到终点的路径：先水平-在垂直路径
std::deque<Point2D> WalkInsideCell(CellNode cell, const Point2D& start,
                                   const Point2D& end) {
  std::deque<Point2D> inner_path = {start};  // 初始化内部路径，起始点为开始点

  // 计算水平和垂直移动的方向和距离
  int delta_x = end.x - start.x;
  int increment_x = (delta_x > 0) ? 1 : -1;  // 确定水平移动方向
  int abs_delta_x = abs(delta_x);            // 水平移动的距离

  int delta_y = end.y - start.y;
  int increment_y = (delta_y > 0) ? 1 : -1;  // 确定垂直移动方向
  int abs_delta_y = abs(delta_y);            // 垂直移动的距离

  // 沿水平方向移动到终点水平位置
  for (int i = 1; i <= abs_delta_x; ++i) {
    inner_path.emplace_back(Point2D(start.x + increment_x * i, start.y));
  }

  // 沿垂直方向移动到终点垂直位置
  for (int i = 1; i <= abs_delta_y; ++i) {
    inner_path.emplace_back(Point2D(end.x, start.y + increment_y * i));
  }

  return inner_path;
}

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
                      int& unvisited_counter, std::deque<CellNode>& path) {
  // 基础情况：如果单元格已经访问过，则直接返回
  if (cell_graph[cell_index].isVisited) {
    return;
  }

  // 标记当前单元格为已访问，并减少未访问单元格计数器
  cell_graph[cell_index].isVisited = true;
  unvisited_counter--;

  // 将当前单元格添加到路径中
  path.emplace_front(cell_graph[cell_index]);

  // 遍历当前单元格的所有邻居
  for (int neighbor_idx : cell_graph[cell_index].neighbor_indices) {
    // 如果邻居单元格未被访问，递归访问它
    if (!cell_graph[neighbor_idx].isVisited) {
      cell_graph[neighbor_idx].parentIndex =
          cell_index;  // 设置当前单元格为邻居的父单元格
      WalkThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
    }
  }

  // 如果所有邻居都已访问，且当前单元格不是起点（没有父单元格），则回溯
  if (unvisited_counter == 0 || cell_graph[cell_index].parentIndex == INT_MAX) {
    return;
  }
}

}  // namespace cover_planning