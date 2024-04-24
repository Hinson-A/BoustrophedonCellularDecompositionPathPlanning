#include <Eigen/Core>
#include <algorithm>
#include <deque>
#include <iostream>
#include <map>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "bcd.h"
#include "cover_planning.h"
#include "map_sdk.h"
#include "math_utils.h"
#include "navigation_message.h"
#include "test_function.h"
#include "visualization.h"

using namespace CommonData;
using namespace cover_planning;
using namespace math_utils;

BoustrophedonCellDecomposition bcd;  //牛耕分解对象

// 注意：在实际使用中，需要在开始遍历之前初始化所有单元格的isVisited为false，parentIndex为INT_MAX。

std::deque<CellNode> GetVisitingPath(std::vector<CellNode>& cell_graph,
                                     int first_cell_index) {
  std::deque<CellNode> visiting_path;

  if (cell_graph.size() == 1) {
    visiting_path.emplace_back(cell_graph.front());
  } else {
    int unvisited_counter = cell_graph.size();
    WalkThroughGraph(cell_graph, first_cell_index, unvisited_counter,
                     visiting_path);
    std::reverse(visiting_path.begin(), visiting_path.end());
  }

  return visiting_path;
}

Point2D FindNextEntrance(const Point2D& curr_point, const CellNode& next_cell,
                         int& corner_indicator) {
  Point2D next_entrance;

  int front_x = next_cell.ceiling.front().x;
  int back_x = next_cell.ceiling.back().x;

  std::vector<Point2D> corner_points = ComputeCellCornerPoints(next_cell);

  if (abs(curr_point.x - front_x) < abs(curr_point.x - back_x)) {
    if (abs(curr_point.y - next_cell.ceiling.front().y) <
        abs(curr_point.y - next_cell.floor.front().y)) {
      next_entrance = corner_points[TOP_LEFT];
      corner_indicator = TOP_LEFT;
    } else {
      next_entrance = corner_points[BOTTOM_LEFT];
      corner_indicator = BOTTOM_LEFT;
    }
  } else {
    if (abs(curr_point.y - next_cell.ceiling.back().y) <
        abs(curr_point.y - next_cell.floor.back().y)) {
      next_entrance = corner_points[TOP_RIGHT];
      corner_indicator = TOP_RIGHT;
    } else {
      next_entrance = corner_points[BOTTOM_RIGHT];
      corner_indicator = BOTTOM_RIGHT;
    }
  }

  return next_entrance;
}

std::deque<std::deque<Point2D>> FindLinkingPath(const Point2D& curr_exit,
                                                Point2D& next_entrance,
                                                int& corner_indicator,
                                                CellNode curr_cell,
                                                const CellNode& next_cell) {
  std::deque<std::deque<Point2D>> path;
  std::deque<Point2D> path_in_curr_cell;
  std::deque<Point2D> path_in_next_cell;

  int exit_corner_indicator = INT_MAX;
  Point2D exit =
      FindNextEntrance(next_entrance, curr_cell, exit_corner_indicator);
  path_in_curr_cell = WalkInsideCell(curr_cell, curr_exit, exit);

  next_entrance = FindNextEntrance(exit, next_cell, corner_indicator);

  int delta_x = next_entrance.x - exit.x;
  int delta_y = next_entrance.y - exit.y;

  int increment_x = 0;
  int increment_y = 0;

  if (delta_x != 0) {
    increment_x = delta_x / std::abs(delta_x);
  }
  if (delta_y != 0) {
    increment_y = delta_y / std::abs(delta_y);
  }

  int upper_bound = INT_MIN;
  int lower_bound = INT_MAX;

  if (exit.x >= curr_cell.ceiling.back().x) {
    upper_bound = curr_cell.ceiling.back().y;
    lower_bound = curr_cell.floor.back().y;
  }
  if (exit.x <= curr_cell.ceiling.front().x) {
    upper_bound = curr_cell.ceiling.front().y;
    lower_bound = curr_cell.floor.front().y;
  }

  if ((next_entrance.y >= upper_bound) && (next_entrance.y <= lower_bound)) {
    for (int y = exit.y; y != next_entrance.y; y += increment_y) {
      path_in_curr_cell.emplace_back(Point2D(exit.x, y));
    }
    for (int x = exit.x; x != next_entrance.x; x += increment_x) {
      path_in_curr_cell.emplace_back(Point2D(x, next_entrance.y));
    }
  } else {
    for (int x = exit.x; x != next_entrance.x; x += increment_x) {
      path_in_curr_cell.emplace_back(Point2D(x, exit.y));
    }
    for (int y = exit.y; y != next_entrance.y; y += increment_y) {
      path_in_next_cell.emplace_back(Point2D(next_entrance.x, y));
    }
  }

  path = {path_in_curr_cell, path_in_next_cell};

  return path;
}

std::deque<Point2D> WalkCrossCells(std::vector<CellNode>& cell_graph,
                                   std::deque<int> cell_path,
                                   const Point2D& start, const Point2D& end,
                                   int robot_radius) {
  std::deque<Point2D> overall_path;
  std::deque<Point2D> sub_path;

  std::deque<std::deque<Point2D>> link_path;

  std::vector<CellNode> cells;
  cells.assign(cell_graph.begin(), cell_graph.end());

  for (auto cell : cells) {
    cell.isCleaned = true;
  }

  Point2D curr_exit, next_entrance;
  int curr_corner_indicator, next_corner_indicator;

  next_entrance =
      FindNextEntrance(start, cells[cell_path[1]], next_corner_indicator);
  curr_exit = FindNextEntrance(next_entrance, cells[cell_path[0]],
                               curr_corner_indicator);
  sub_path = WalkInsideCell(cells[cell_path[0]], start, curr_exit);
  overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
  sub_path.clear();

  link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator,
                              cells[cell_path[0]], cells[cell_path[1]]);
  sub_path.insert(sub_path.end(), link_path.front().begin(),
                  link_path.front().end());
  sub_path.insert(sub_path.end(), link_path.back().begin(),
                  link_path.back().end());

  overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
  sub_path.clear();

  curr_corner_indicator = next_corner_indicator;

  for (int i = 1; i < cell_path.size() - 1; i++) {
    sub_path = GetBoustrophedonPath(cell_graph, cells[cell_path[i]],
                                    curr_corner_indicator, robot_radius);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    curr_exit = overall_path.back();
    next_entrance = FindNextEntrance(curr_exit, cells[cell_path[i + 1]],
                                     next_corner_indicator);

    link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator,
                                cells[cell_path[i]], cells[cell_path[i + 1]]);
    sub_path.insert(sub_path.end(), link_path.front().begin(),
                    link_path.front().end());
    sub_path.insert(sub_path.end(), link_path.back().begin(),
                    link_path.back().end());

    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    curr_corner_indicator = next_corner_indicator;
  }

  sub_path = WalkInsideCell(cells[cell_path.back()], next_entrance, end);
  overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
  sub_path.clear();

  return overall_path;
}

/** 广度优先搜索 **/
std::deque<int> FindShortestPath(std::vector<CellNode>& cell_graph,
                                 const Point2D& start, const Point2D& end) {
  int start_cell_index = DetermineCellIndex(cell_graph, start).front();
  int end_cell_index = DetermineCellIndex(cell_graph, end).front();

  std::deque<int> cell_path = {end_cell_index};

  if (start_cell_index == end_cell_index) {
    return cell_path;
  }

  if (start_cell_index == end_cell_index) {
    cell_path.emplace_back(start_cell_index);
    return cell_path;
  }

  std::vector<CellNode> cells;
  cells.assign(cell_graph.begin(), cell_graph.end());

  for (auto cell : cells) {
    cell.isVisited = false;
    cell.isCleaned = false;
    cell.parentIndex = INT_MAX;
  }

  std::deque<int> search_queue = {start_cell_index};

  CellNode curr_cell;

  while (!search_queue.empty()) {
    curr_cell = cells[search_queue.front()];

    cells[search_queue.front()].isVisited = true;
    search_queue.pop_front();

    for (int i = 0; i < curr_cell.neighbor_indices.size(); i++) {
      if (curr_cell.neighbor_indices[i] == end_cell_index) {
        cells[curr_cell.neighbor_indices[i]].parentIndex = curr_cell.cellIndex;
        search_queue.clear();
        break;
      } else if (!cells[curr_cell.neighbor_indices[i]].isVisited) {
        cells[curr_cell.neighbor_indices[i]].isVisited = true;
        cells[curr_cell.neighbor_indices[i]].parentIndex = curr_cell.cellIndex;
        search_queue.emplace_back(curr_cell.neighbor_indices[i]);
      }
    }
  }

  curr_cell = cells[end_cell_index];
  int prev_cell_index;

  while (curr_cell.parentIndex != INT_MAX) {
    prev_cell_index = curr_cell.parentIndex;
    cell_path.emplace_front(prev_cell_index);
    curr_cell = cells[prev_cell_index];
  }

  return cell_path;
}

/** 静态路径规划流程 **/

std::deque<std::deque<Point2D>> StaticPathPlanning(
    const cv::Mat& map, std::vector<CellNode>& cell_graph,
    const Point2D& start_point, int robot_radius, bool visualize_cells,
    bool visualize_path, int color_repeats = 10) {
  cv::Mat3b vis_map;
  cv::cvtColor(map, vis_map, cv::COLOR_GRAY2BGR);

  std::deque<std::deque<Point2D>> global_path;
  std::deque<Point2D> local_path;
  int corner_indicator = TOP_LEFT;

  //确定起始点所在的cell索引
  int start_cell_index = DetermineCellIndex(cell_graph, start_point).front();

  //从当前位置到cell内的起始角点
  std::deque<Point2D> init_path = WalkInsideCell(
      cell_graph[start_cell_index], start_point,
      ComputeCellCornerPoints(cell_graph[start_cell_index])[TOP_LEFT]);

  local_path.assign(init_path.begin(), init_path.end());

  std::deque<CellNode> cell_path =
      GetVisitingPath(cell_graph, start_cell_index);

  if (visualize_cells || visualize_path) {
    cv::namedWindow("visualize_path", cv::WINDOW_NORMAL);
    cv::imshow("visualize_path", vis_map);
  }

  if (visualize_cells) {
    std::cout << "cell graph has " << cell_graph.size() << " cells."
              << std::endl;
    for (int i = 0; i < cell_graph.size(); i++) {
      for (int j = 0; j < cell_graph[i].neighbor_indices.size(); j++) {
        std::cout << "cell " << i << "'s neighbor: cell "
                  << cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex
                  << std::endl;
      }
    }

    for (const auto& cell : cell_graph) {
      map_visualization::DrawCells(vis_map, cell);
      cv::imshow("map", vis_map);
      cv::waitKey(500);
    }
  }

  std::deque<cv::Scalar> JetColorMap;
  map_visualization::InitializeColorMap(JetColorMap, color_repeats);

  if (visualize_path) {
    cv::circle(vis_map, cv::Point(start_point.x, start_point.y), 1,
               cv::Scalar(0, 0, 255), -1);
    for (const auto& point : init_path) {
      vis_map.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(
          uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
          uchar(JetColorMap.front()[2]));
      map_visualization::UpdateColorMap(JetColorMap);
      cv::imshow("map", vis_map);
      cv::waitKey(1);
    }
  }

  std::deque<Point2D> inner_path;
  std::deque<std::deque<Point2D>> link_path;
  Point2D curr_exit;
  Point2D next_entrance;

  std::deque<int> return_cell_path;
  std::deque<Point2D> return_path;

  for (int i = 0; i < cell_path.size(); i++) {
    inner_path = GetBoustrophedonPath(cell_graph, cell_path[i],
                                      corner_indicator, robot_radius);
    local_path.insert(local_path.end(), inner_path.begin(), inner_path.end());
    if (visualize_path) {
      for (const auto& point : inner_path) {
        vis_map.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(
            uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
            uchar(JetColorMap.front()[2]));
        map_visualization::UpdateColorMap(JetColorMap);
        cv::imshow("map", vis_map);
        cv::waitKey(1);
      }
    }

    cell_graph[cell_path[i].cellIndex].isCleaned = true;

    if (i < (cell_path.size() - 1)) {
      curr_exit = inner_path.back();
      next_entrance =
          FindNextEntrance(curr_exit, cell_path[i + 1], corner_indicator);
      link_path = FindLinkingPath(curr_exit, next_entrance, corner_indicator,
                                  cell_path[i], cell_path[i + 1]);

      // for debugging
      //            std::cout<<std::endl;
      //            for(int i = 0; i < link_path.front().size(); i++)
      //            {
      //                int idx = DetermineCellIndex(cell_graph,
      //                link_path.front()[i]).front(); std::cout<<"point lies in
      //                curr cell "<<idx<<std::endl;
      //            }
      //
      //            for(int i = 0; i < link_path.back().size(); i++)
      //            {
      //                int idx = DetermineCellIndex(cell_graph,
      //                link_path.back()[i]).front(); std::cout<<"point lies in
      //                next cell "<<idx<<std::endl;
      //            }
      //            std::cout<<std::endl;

      local_path.insert(local_path.end(), link_path.front().begin(),
                        link_path.front().end());
      global_path.emplace_back(local_path);
      local_path.clear();
      local_path.insert(local_path.end(), link_path.back().begin(),
                        link_path.back().end());

      if (visualize_path) {
        for (const auto& point : link_path.front()) {
          //                    vis_map.at<cv::Vec3b>(point.y,
          //                    point.x)=cv::Vec3b(255, 255, 255);
          vis_map.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(
              uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
              uchar(JetColorMap.front()[2]));
          map_visualization::UpdateColorMap(JetColorMap);
          cv::imshow("map", vis_map);
          cv::waitKey(1);
        }

        for (const auto& point : link_path.back()) {
          //                    vis_map.at<cv::Vec3b>(point.y,
          //                    point.x)=cv::Vec3b(255, 255, 255);
          vis_map.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(
              uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
              uchar(JetColorMap.front()[2]));
          map_visualization::UpdateColorMap(JetColorMap);
          cv::imshow("map", vis_map);
          cv::waitKey(1);
        }
      }
    }
  }
  global_path.emplace_back(local_path);

  if (visualize_cells || visualize_path) {
    cv::waitKey(0);
  }

  return global_path;
}

std::deque<Point2D> ReturningPathPlanning(
    cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& curr_pos,
    const Point2D& original_pos, int robot_radius, bool visualize_path) {
  std::deque<int> return_cell_path =
      FindShortestPath(cell_graph, curr_pos, original_pos);
  std::deque<Point2D> returning_path;

  if (return_cell_path.size() == 1) {
    returning_path = WalkInsideCell(cell_graph[return_cell_path.front()],
                                    curr_pos, original_pos);
  } else {
    returning_path = WalkCrossCells(cell_graph, return_cell_path, curr_pos,
                                    original_pos, robot_radius);
  }

  if (visualize_path) {
    cv::namedWindow("map", cv::WINDOW_NORMAL);
    for (const auto& point : returning_path) {
      map.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(250, 250, 250);
      cv::imshow("map", map);
      cv::waitKey(1);
    }
    cv::waitKey(0);
  }

  return returning_path;
}

/** 生成运动指令 **/

/** 动态路径规划（未完成） **/

// 待重写
const int UP = 0, UPRIGHT = 1, RIGHT = 2, DOWNRIGHT = 3, DOWN = 4, DOWNLEFT = 5,
          LEFT = 6, UPLEFT = 7, CENTER = 8;
const std::vector<int> map_directions = {UP,   UPRIGHT,  RIGHT, DOWNRIGHT,
                                         DOWN, DOWNLEFT, LEFT,  UPLEFT};

int GetFrontDirection(const Point2D& curr_pos, const Point2D& next_pos) {
  int delta_x = next_pos.x - curr_pos.x;
  int delta_y = next_pos.y - curr_pos.y;

  if (delta_y < 0) {
    if (delta_x == 0) {
      return UP;
    }
    if (delta_x < 0) {
      return UPLEFT;
    }
    if (delta_x > 0) {
      return UPRIGHT;
    }
  }

  if (delta_y > 0) {
    if (delta_x == 0) {
      return DOWN;
    }
    if (delta_x < 0) {
      return DOWNLEFT;
    }
    if (delta_x > 0) {
      return DOWNRIGHT;
    }
  }

  if (delta_y == 0) {
    if (delta_x == 0) {
      return CENTER;
    }
    if (delta_x < 0) {
      return LEFT;
    }
    if (delta_x > 0) {
      return RIGHT;
    }
  }
}

int GetBackDirection(int front_direction) {
  if (front_direction + 4 >= map_directions.size()) {
    int index_offset = front_direction + 4 - map_directions.size();
    return map_directions[index_offset];
  } else {
    return map_directions[front_direction + 4];
  }
}

int GetLeftDirection(int front_direction) {
  if (front_direction - 2 < 0) {
    int index_offset = 2 - front_direction;
    return map_directions[map_directions.size() - index_offset];
  } else {
    return map_directions[front_direction - 2];
  }
}

int GetRightDirection(int front_direction) {
  if (front_direction + 2 >= map_directions.size()) {
    int index_offset = front_direction + 2 - map_directions.size();
    return map_directions[index_offset];
  } else {
    return map_directions[front_direction + 2];
  }
}

// 模拟机器人尝试旋转, 旋转方向都是顺时针排列
std::vector<int> GetFrontDirectionCandidates(int front_direction) {
  std::vector<int> front_directions;

  if (front_direction - 2 < 0) {
    int index_offset = 2 - front_direction;
    front_directions.emplace_back(
        map_directions[map_directions.size() - index_offset]);
  } else {
    front_directions.emplace_back(map_directions[front_direction - 2]);
  }

  if (front_direction - 1 < 0) {
    int index_offset = 1 - front_direction;
    front_directions.emplace_back(
        map_directions[map_directions.size() - index_offset]);
  } else {
    front_directions.emplace_back(map_directions[front_direction - 1]);
  }

  front_directions.emplace_back(map_directions[front_direction]);

  if (front_direction + 1 >= map_directions.size()) {
    int index_offset = front_direction + 1 - map_directions.size();
    front_directions.emplace_back(map_directions[index_offset]);
  } else {
    front_directions.emplace_back(map_directions[front_direction + 1]);
  }

  if (front_direction + 2 >= map_directions.size()) {
    int index_offset = front_direction + 2 - map_directions.size();
    front_directions.emplace_back(map_directions[index_offset]);
  } else {
    front_directions.emplace_back(map_directions[front_direction + 2]);
  }

  return front_directions;
}

std::vector<int> GetBackDirectionCandidates(int front_direction) {
  std::vector<int> back_directions;
  int first_direction = GetRightDirection(front_direction);

  for (int i = 0; i <= 4; i++) {
    if (first_direction + i >= map_directions.size()) {
      int index_offset = first_direction + i - map_directions.size();
      back_directions.emplace_back(map_directions[index_offset]);
    } else {
      back_directions.emplace_back(map_directions[first_direction + i]);
    }
  }

  return back_directions;
}

std::vector<int> GetLeftDirectionCandidates(int front_direction) {
  std::vector<int> left_directions;

  for (int i = 4; i >= 0; i--) {
    if (front_direction - i < 0) {
      int index_offset = i - front_direction;
      left_directions.emplace_back(
          map_directions[map_directions.size() - index_offset]);
    } else {
      left_directions.emplace_back(map_directions[front_direction - i]);
    }
  }

  return left_directions;
}

std::vector<int> GetRightDirectionCandidates(int front_direction) {
  std::vector<int> right_directions;

  for (int i = 0; i <= 4; i++) {
    if (front_direction + i >= map_directions.size()) {
      int index_offset = front_direction + i - map_directions.size();
      right_directions.emplace_back(map_directions[index_offset]);
    } else {
      right_directions.emplace_back(map_directions[front_direction + i]);
    }
  }

  return right_directions;
}

Point2D GetNextPosition(const Point2D& curr_pos, int direction, int steps) {
  Point2D next_position;

  switch (direction) {
    case UP:
      next_position.x = int(curr_pos.x);
      next_position.y = int(curr_pos.y - steps);
      return next_position;
    case UPRIGHT:
      next_position.x = int(curr_pos.x + steps);
      next_position.y = int(curr_pos.y - steps);
      return next_position;
    case RIGHT:
      next_position.x = int(curr_pos.x + steps);
      next_position.y = int(curr_pos.y);
      return next_position;
    case DOWNRIGHT:
      next_position.x = int(curr_pos.x + steps);
      next_position.y = int(curr_pos.y + steps);
      return next_position;
    case DOWN:
      next_position.x = int(curr_pos.x);
      next_position.y = int(curr_pos.y + steps);
      return next_position;
    case DOWNLEFT:
      next_position.x = int(curr_pos.x - steps);
      next_position.y = int(curr_pos.y + steps);
      return next_position;
    case LEFT:
      next_position.x = int(curr_pos.x - steps);
      next_position.y = int(curr_pos.y);
      return next_position;
    case UPLEFT:
      next_position.x = int(curr_pos.x - steps);
      next_position.y = int(curr_pos.y - steps);
      return next_position;
    case CENTER:
      next_position.x = int(curr_pos.x);
      next_position.y = int(curr_pos.y);
      return next_position;
    default:
      return next_position;
  }
}

// 模拟碰撞传感器信号
bool CollisionOccurs(const cv::Mat& map, const Point2D& curr_pos,
                     int detect_direction, int robot_radius) {
  int obstacle_dist = INT_MAX;
  Point2D ray_pos;

  for (int i = 1; i <= (robot_radius + 1); i++) {
    ray_pos = GetNextPosition(curr_pos, detect_direction, i);

    if (ray_pos.x < 0 || ray_pos.y < 0 || ray_pos.x >= map.cols ||
        ray_pos.y >= map.rows) {
      break;
    }

    if (map.at<cv::Vec3b>(ray_pos.y, ray_pos.x) == cv::Vec3b(255, 255, 255)) {
      obstacle_dist = i;
      break;
    }
  }

  return (obstacle_dist == (robot_radius + 1));
}

// 结束返回false, 继续则返回true
bool WalkAlongObstacle(const cv::Mat& map, const Point2D& obstacle_origin,
                       const Point2D& contouring_origin,
                       int detecting_direction,
                       const std::vector<int>& direction_candidates,
                       Point2D& curr_pos, int first_turning_direction,
                       int second_turning_direction, Polygon& obstacle,
                       Polygon& new_obstacle, bool& isObstacleCompleted,
                       std::deque<Point2D>& contouring_path, int robot_radius) {
  bool turning = false;
  Point2D last_curr_pos = curr_pos;
  Point2D next_pos;
  Point2D obstacle_point;

  while (!turning) {
    for (auto direction : direction_candidates) {
      next_pos = GetNextPosition(curr_pos, direction, 1);
      if (next_pos.x < 0 || next_pos.y < 0 || next_pos.x >= map.cols ||
          next_pos.y >= map.rows) {
        continue;
      }

      if (CollisionOccurs(map, next_pos, detecting_direction, robot_radius)) {
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
        obstacle_point =
            GetNextPosition(next_pos, detecting_direction, robot_radius + 1);
        if (obstacle_point.x == obstacle_origin.x &&
            obstacle_point.y == obstacle_origin.y) {
          if (!isObstacleCompleted) {
            obstacle.assign(new_obstacle.begin(), new_obstacle.end());
            isObstacleCompleted = true;
          }
        }
        new_obstacle.emplace_back(obstacle_point);
        break;
      }
    }
    if (curr_pos.x == last_curr_pos.x && curr_pos.y == last_curr_pos.y) {
      turning = true;
    } else {
      last_curr_pos = curr_pos;
    }

    if (std::find(contouring_path.begin(), (contouring_path.end() - 1),
                  next_pos) != (contouring_path.end() - 1) &&
        contouring_path.size() > 1 && isObstacleCompleted) {
      return false;
    }
  }
  for (int i = 1; i <= (robot_radius + 1); i++) {
    next_pos = GetNextPosition(curr_pos, first_turning_direction, 1);
    contouring_path.emplace_back(next_pos);
    curr_pos = next_pos;

    if (std::find(contouring_path.begin(), (contouring_path.end() - 1),
                  next_pos) != (contouring_path.end() - 1) &&
        contouring_path.size() > 1 && isObstacleCompleted) {
      return false;
    }
  }
  for (int i = 1; i <= (robot_radius + 1); i++) {
    next_pos = GetNextPosition(curr_pos, second_turning_direction, 1);
    contouring_path.emplace_back(next_pos);
    curr_pos = next_pos;

    if (std::find(contouring_path.begin(), (contouring_path.end() - 1),
                  next_pos) != (contouring_path.end() - 1) &&
        contouring_path.size() > 1 && isObstacleCompleted) {
      return false;
    }
  }
  return true;
}

Polygon GetNewObstacle(const cv::Mat& map, Point2D origin, int front_direction,
                       std::deque<Point2D>& contouring_path, int robot_radius) {
  contouring_path.emplace_back(origin);

  Point2D curr_pos = origin;

  Point2D obstacle_point;
  Polygon new_obstacle;
  Polygon obstacle;

  bool isObstacleCompleted = false;

  int left_direction = GetLeftDirection(front_direction);
  int right_direction = GetRightDirection(front_direction);
  int back_direction = GetBackDirection(front_direction);

  std::deque<int> direcition_list = {right_direction, front_direction,
                                     left_direction, back_direction};

  std::vector<int> right_direction_candidates =
      GetRightDirectionCandidates(front_direction);
  std::vector<int> front_direction_candidates =
      GetFrontDirectionCandidates(front_direction);
  std::vector<int> left_direction_candidates =
      GetLeftDirectionCandidates(front_direction);
  std::vector<int> back_direction_candidates =
      GetBackDirectionCandidates(front_direction);

  std::deque<std::vector<int>> direction_candidates_list = {
      right_direction_candidates, front_direction_candidates,
      left_direction_candidates, back_direction_candidates};

  obstacle_point = GetNextPosition(origin, front_direction, robot_radius + 1);
  new_obstacle.emplace_back(obstacle_point);

  Point2D obstacle_origin = new_obstacle.front();

  int detecting_direction;
  int first_turning_direction;
  int second_turning_direction;
  int temp_direction;

  bool keepContouring = true;
  std::vector<int> direction_candidates;
  std::vector<int> temp_direction_candidates;

  while (keepContouring) {
    direction_candidates = direction_candidates_list[0];
    first_turning_direction = direcition_list[0];
    second_turning_direction = direcition_list[1];
    detecting_direction = direcition_list[1];

    keepContouring = WalkAlongObstacle(
        map, obstacle_origin, origin, detecting_direction, direction_candidates,
        curr_pos, first_turning_direction, second_turning_direction, obstacle,
        new_obstacle, isObstacleCompleted, contouring_path, robot_radius);

    temp_direction = direcition_list.front();
    direcition_list.pop_front();
    direcition_list.emplace_back(temp_direction);

    temp_direction_candidates = direction_candidates_list.front();
    direction_candidates_list.pop_front();
    direction_candidates_list.emplace_back(temp_direction_candidates);
  }

  contouring_path.pop_front();

  return obstacle;

}  // 考虑使用contouring path当障碍物

int GetCleaningDirection(const CellNode& cell, Point2D exit) {
  std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell);

  //    if(exit.x == corner_points[TOP_LEFT].x && exit.y ==
  //    corner_points[TOP_LEFT].y)
  //    {
  //        return LEFT;
  //    }
  //    if(exit.x == corner_points[BOTTOM_LEFT].x && exit.y ==
  //    corner_points[BOTTOM_LEFT].y)
  //    {
  //        return LEFT;
  //    }
  //    if(exit.x == corner_points[TOP_RIGHT].x && exit.y ==
  //    corner_points[TOP_RIGHT].y)
  //    {
  //        return RIGHT;
  //    }
  //    if(exit.x == corner_points[BOTTOM_RIGHT].x && exit.y ==
  //    corner_points[BOTTOM_RIGHT].y)
  //    {
  //        return RIGHT;
  //    }

  double dist_to_left = std::abs(exit.x - corner_points[TOP_LEFT].x);
  double dist_to_right = std::abs(exit.x = corner_points[TOP_RIGHT].x);

  if (dist_to_left >= dist_to_right) {
    return RIGHT;
  } else {
    return LEFT;
  }
}

// 清扫方向只分向左和向右
std::deque<std::deque<Point2D>> LocalReplanning(
    cv::Mat& map, CellNode outer_cell, const PolygonList& obstacles,
    const Point2D& curr_pos, std::vector<CellNode>& curr_cell_graph,
    int cleaning_direction, int robot_radius, bool visualize_cells = false,
    bool visualize_path = false) {
  // TODO: 边界判断
  int start_x = INT_MAX;
  int end_x = INT_MAX;

  if (cleaning_direction == LEFT) {
    start_x = outer_cell.ceiling.front().x;

    if (curr_pos.x + 2 * (robot_radius + 1) <= outer_cell.ceiling.back().x) {
      end_x = curr_pos.x + 2 * (robot_radius + 1);
    } else {
      end_x = outer_cell.ceiling.back().x;
    }
  }
  if (cleaning_direction == RIGHT) {
    end_x = outer_cell.ceiling.back().x;

    if (curr_pos.x - 2 * (robot_radius + 1) >= outer_cell.ceiling.front().x) {
      start_x = curr_pos.x - 2 * (robot_radius + 1);
    } else {
      start_x = outer_cell.ceiling.front().x;
    }
  }
  int outer_cell_start_index_offset = start_x - outer_cell.ceiling.front().x;
  int outer_cell_end_index_offset = end_x - outer_cell.ceiling.front().x;

  CellNode inner_cell;
  for (int i = outer_cell_start_index_offset; i <= outer_cell_end_index_offset;
       i++) {
    inner_cell.ceiling.emplace_back(outer_cell.ceiling[i]);
    inner_cell.floor.emplace_back(outer_cell.floor[i]);
  }

  //    curr_cell_graph = GenerateCells(map, inner_cell, obstacles);   //
  //    这里需要改
  std::deque<std::deque<Point2D>> replanning_path =
      StaticPathPlanning(map, curr_cell_graph, curr_pos, robot_radius,
                         visualize_cells, visualize_path);

  return replanning_path;
}  // 回退区域需要几个r+1

// 每一段都是在一个cell中的路径
std::deque<Point2D> DynamicPathPlanning(
    cv::Mat& map, const std::vector<CellNode>& global_cell_graph,
    std::deque<std::deque<Point2D>> global_path, int robot_radius,
    bool returning_home, bool visualize_path, int color_repeats = 10) {
  std::deque<Point2D> dynamic_path;

  std::deque<std::deque<Point2D>> curr_path;
  std::deque<Point2D> curr_sub_path;
  std::deque<Point2D> contouring_path;

  std::deque<std::deque<Point2D>> replanning_path;
  std::deque<std::deque<Point2D>> remaining_curr_path;

  std::deque<Point2D> linking_path;

  Point2D curr_pos;
  Point2D next_pos;
  Point2D curr_exit;

  int front_direction;
  int cleaning_direction;

  Polygon new_obstacle;
  //    Polygon temp_new_obstacle;

  int curr_cell_index;
  CellNode curr_cell;

  std::vector<CellNode> curr_cell_graph;

  PolygonList overall_obstacles;
  PolygonList curr_obstacles;
  std::vector<cv::Point> visited_obstacle_contour;
  std::vector<std::vector<cv::Point>> visited_obstacle_contours;

  std::vector<std::deque<std::deque<Point2D>>> unvisited_paths = {global_path};
  std::vector<std::vector<CellNode>> cell_graph_list = {global_cell_graph};
  std::vector<Point2D> exit_list = {global_path.back().back()};

  cv::Mat vismap = map.clone();
  std::deque<cv::Scalar> JetColorMap;
  map_visualization::InitializeColorMap(JetColorMap, color_repeats);
  if (visualize_path) {
    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map", vismap);
  }

  while (!unvisited_paths.empty() && !cell_graph_list.empty()) {
    curr_path = unvisited_paths.back();
    curr_cell_graph = cell_graph_list.back();

    for (int i = 0; i < curr_path.size(); i++) {
      curr_sub_path.assign(curr_path[i].begin(), curr_path[i].end());

      for (int j = 0; j < curr_sub_path.size() - 1; j++) {
        curr_pos = curr_sub_path[j];
        next_pos = curr_sub_path[j + 1];
        dynamic_path.emplace_back(curr_pos);

        if (visualize_path) {
          vismap.at<cv::Vec3b>(curr_pos.y, curr_pos.x) = cv::Vec3b(
              uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
              uchar(JetColorMap.front()[2]));
          map_visualization::UpdateColorMap(JetColorMap);
          cv::imshow("map", vismap);
          cv::waitKey(1);
        }

        front_direction = GetFrontDirection(curr_pos, next_pos);
        if (CollisionOccurs(map, curr_pos, front_direction, robot_radius)) {
          new_obstacle = GetNewObstacle(map, curr_pos, front_direction,
                                        contouring_path, robot_radius);
          //                    new_obstacle = GetSingleContouringArea(map,
          //                    temp_new_obstacle, robot_radius);
          overall_obstacles.emplace_back(new_obstacle);

          // for debugging
          //                    for(int i = 0; i < new_obstacle.size(); i++)
          //                    {
          //                        std::cout<<"x:"<< new_obstacle[i].x <<",
          //                        y:"<< new_obstacle[i].y <<std::endl;
          //                    }
          //
          //                    for(int i = 0; i < new_obstacle.size(); i++)
          //                    {
          //                        vismap.at<cv::Vec3b>(new_obstacle[i].y,
          //                        new_obstacle[i].x)=cv::Vec3b(0, 255, 0);
          //                    }
          //                    for(int i = 0; i < contouring_path.size(); i++)
          //                    {
          //                        vismap.at<cv::Vec3b>(contouring_path[i].y,
          //                        contouring_path[i].x)=cv::Vec3b(255, 0, 0);
          //                    }
          //                    cv::circle(map,
          //                    cv::Point(contouring_path.front().x,
          //                    contouring_path.front().y), 2, cv::Scalar(0,
          //                    255, 255), -1); cv::circle(map,
          //                    cv::Point(contouring_path.back().x,
          //                    contouring_path.back().y), 2, cv::Scalar(255, 0,
          //                    255), -1); PointTypeTest(map, new_obstacle);
          //                    cv::imshow("map", vismap);
          //                    cv::waitKey(0);

          dynamic_path.insert(dynamic_path.end(), contouring_path.begin(),
                              contouring_path.end());

          if (visualize_path) {
            for (const auto& point : contouring_path) {
              vismap.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(
                  uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
                  uchar(JetColorMap.front()[2]));
              map_visualization::UpdateColorMap(JetColorMap);
              cv::imshow("map", vismap);
              cv::waitKey(1);
            }
          }

          contouring_path.clear();

          visited_obstacle_contour.clear();
          visited_obstacle_contours.clear();

          for (const auto& point : new_obstacle) {
            visited_obstacle_contour.emplace_back(cv::Point(point.x, point.y));
          }

          visited_obstacle_contours.emplace_back(visited_obstacle_contour);

          curr_cell_index =
              DetermineCellIndex(curr_cell_graph, curr_pos).front();
          curr_cell = curr_cell_graph[curr_cell_index];
          curr_obstacles = {new_obstacle};
          curr_exit = curr_sub_path.back();

          // for debugging
          //                    map_visualization::DrawCells(map, curr_cell,
          //                    cv::Scalar(255, 0, 255));
          //                    vismap.at<cv::Vec3b>(curr_exit.y,
          //                    curr_exit.x)=cv::Vec3b(0,255,255);
          //                    cv::imshow("map", vismap);
          //                    cv::waitKey(0);

          cleaning_direction = GetCleaningDirection(curr_cell, curr_exit);

          replanning_path = LocalReplanning(
              map, curr_cell, curr_obstacles, dynamic_path.back(),
              curr_cell_graph, cleaning_direction, robot_radius, false,
              false);  // 此处会更新curr_cell_graph
          cv::fillPoly(map, visited_obstacle_contours, cv::Scalar(50, 50, 50));
          cv::fillPoly(vismap, visited_obstacle_contours,
                       cv::Scalar(50, 50, 50));

          remaining_curr_path.assign(curr_path.begin() + i + 1,
                                     curr_path.end());

          goto UPDATING_REMAINING_PATHS;
        }
      }
    }

    if (dynamic_path.back().x != exit_list.back().x &&
        dynamic_path.back().y != exit_list.back().y) {
      linking_path = ReturningPathPlanning(
          map, cell_graph_list.back(), dynamic_path.back(), exit_list.back(),
          robot_radius, false);
      dynamic_path.insert(dynamic_path.end(), linking_path.begin(),
                          linking_path.end());

      if (visualize_path) {
        for (const auto& point : linking_path) {
          vismap.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(
              uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
              uchar(JetColorMap.front()[2]));
          map_visualization::UpdateColorMap(JetColorMap);
          cv::imshow("map", vismap);
          cv::waitKey(1);
        }
      }
    }

    exit_list.pop_back();
    unvisited_paths.pop_back();
    cell_graph_list.pop_back();
    continue;

  UPDATING_REMAINING_PATHS:
    exit_list.emplace_back(curr_exit);
    unvisited_paths.pop_back();
    unvisited_paths.emplace_back(remaining_curr_path);
    unvisited_paths.emplace_back(replanning_path);
    cell_graph_list.emplace_back(curr_cell_graph);
  }

  if (returning_home) {
    cv::Mat3b returning_map = cv::Mat3b(map.size(), CV_8U);
    returning_map.setTo(cv::Scalar(0, 0, 0));
    std::vector<cv::Point> returning_obstacle_contour;
    std::vector<std::vector<cv::Point>> returning_obstacle_contours;

    for (const auto& obstacle : overall_obstacles) {
      for (const auto& point : obstacle) {
        returning_obstacle_contour.emplace_back(cv::Point(point.x, point.y));
      }
      returning_obstacle_contours.emplace_back(returning_obstacle_contour);
      returning_obstacle_contour.clear();
    }

    cv::fillPoly(returning_map, returning_obstacle_contours,
                 cv::Scalar(255, 255, 255));

    Polygon returning_map_border = map_sdk::ConstructDefaultWall(returning_map);
    //        std::vector<CellNode> returning_cell_graph =
    //        GenerateCells(returning_map, returning_map_border,
    //        overall_obstacles);//这里需要改
    std::vector<CellNode> returning_cell_graph;  // 这里需要改

    // for debugging
    //        for(auto cell:returning_cell_graph)
    //        {
    //            map_visualization::DrawCells(vismap, cell, cv::Scalar(0, 255,
    //            255)); cv::imshow("map", vismap); cv::waitKey(0);
    //        }
    //

    for (auto cell : returning_cell_graph) {
      cell.isCleaned = true;
    }

    std::deque<Point2D> returning_path = ReturningPathPlanning(
        returning_map, returning_cell_graph, dynamic_path.back(),
        dynamic_path.front(), robot_radius, false);

    if (visualize_path) {
      for (const auto& point : returning_path) {
        vismap.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(250, 250, 250);
        cv::imshow("map", vismap);
        cv::waitKey(1);
      }
    }

    dynamic_path.insert(dynamic_path.end(), returning_path.begin(),
                        returning_path.end());
  }

  if (visualize_path) {
    cv::waitKey(5000);
  }

  return dynamic_path;
}

void MoveAsPathPlannedTest(
    cv::Mat& map, double meters_per_pix, const Point2D& start,
    const std::vector<NavigationMessage>& motion_commands) {
  int pixs;
  Point2D begin = start, end;

  cv::namedWindow("original_map", cv::WINDOW_NORMAL);

  for (auto command : motion_commands) {
    pixs = int(command.GetDistance() / meters_per_pix);
    if (command.GetGlobalYaw() == 0.0) {
      end = Point2D(begin.x, begin.y - pixs);
      cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y),
               cv::Scalar(0, 0, 255));
      cv::imshow("original_map", map);
      cv::waitKey(100);
    }
    if (command.GetGlobalYaw() == 180.0) {
      end = Point2D(begin.x, begin.y + pixs);
      cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y),
               cv::Scalar(255, 0, 0));
      cv::imshow("original_map", map);
      cv::waitKey(100);
    }
    if (command.GetGlobalYaw() == 90.0) {
      end = Point2D(begin.x + pixs, begin.y);
      cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y),
               cv::Scalar(0, 255, 0));
      cv::imshow("original_map", map);
      cv::waitKey(100);
    }
    if (command.GetGlobalYaw() == -90.0) {
      end = Point2D(begin.x - pixs, begin.y);
      cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y),
               cv::Scalar(0, 255, 255));
      cv::imshow("original_map", map);
      cv::waitKey(100);
    }
    begin = end;
  }
  cv::waitKey(0);
}

/** 测试辅助函数 **/

void CheckObstaclePointType(cv::Mat& map, const Polygon& obstacle) {
  PolygonList obstacles = {obstacle};

  std::vector<Event> event_list = bcd.GenerateObstacleEventList(map, obstacles);

  for (auto event : event_list) {
    if (event.event_type == IN) {
      std::cout << event.x << ", " << event.y << ", IN" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == IN_TOP) {
      std::cout << event.x << ", " << event.y << ", IN_TOP" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == IN_BOTTOM) {
      std::cout << event.x << ", " << event.y << ", IN_BOTTOM" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == OUT) {
      std::cout << event.x << ", " << event.y << ", OUT" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == OUT_TOP) {
      std::cout << event.x << ", " << event.y << ", OUT_TOP" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == OUT_BOTTOM) {
      std::cout << event.x << ", " << event.y << ", OUT_BOTTOM" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == INNER_IN) {
      std::cout << event.x << ", " << event.y << ", INNER_IN" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == INNER_IN_TOP) {
      std::cout << event.x << ", " << event.y << ", INNER_IN_TOP" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == INNER_IN_BOTTOM) {
      std::cout << event.x << ", " << event.y << ", INNER_IN_BOTTOM"
                << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == INNER_OUT) {
      std::cout << event.x << ", " << event.y << ", INNER_OUT" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == INNER_OUT_TOP) {
      std::cout << event.x << ", " << event.y << ", INNER_OUT_TOP" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == INNER_OUT_BOTTOM) {
      std::cout << event.x << ", " << event.y << ", INNER_OUT_BOTTOM"
                << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == MIDDLE) {
      map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50, 50);
    }
    if (event.event_type == CEILING) {
      map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255, 255);
    }
    if (event.event_type == FLOOR) {
      map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0, 0);
    }
  }
}

void CheckWallPointType(cv::Mat& map, const Polygon& wall) {
  std::vector<Event> event_list = bcd.GenerateWallEventList(map, wall);
  for (auto event : event_list) {
    if (event.event_type == IN_EX) {
      std::cout << event.x << ", " << event.y << ", IN_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == IN_TOP_EX) {
      std::cout << event.x << ", " << event.y << ", IN_TOP_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == IN_BOTTOM_EX) {
      std::cout << event.x << ", " << event.y << ", IN_BOTTOM_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == OUT_EX) {
      std::cout << event.x << ", " << event.y << ", OUT_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == OUT_TOP_EX) {
      std::cout << event.x << ", " << event.y << ", OUT_TOP_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == OUT_BOTTOM_EX) {
      std::cout << event.x << ", " << event.y << ", OUT_BOTTOM_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == INNER_IN_EX) {
      std::cout << event.x << ", " << event.y << ", INNER_IN_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == INNER_IN_TOP_EX) {
      std::cout << event.x << ", " << event.y << ", INNER_IN_TOP_EX"
                << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == INNER_IN_BOTTOM_EX) {
      std::cout << event.x << ", " << event.y << ", INNER_IN_BOTTOM_EX"
                << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (event.event_type == INNER_OUT_EX) {
      std::cout << event.x << ", " << event.y << ", INNER_OUT_EX" << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == INNER_OUT_TOP_EX) {
      std::cout << event.x << ", " << event.y << ", INNER_OUT_TOP_EX"
                << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == INNER_OUT_BOTTOM_EX) {
      std::cout << event.x << ", " << event.y << ", INNER_OUT_BOTTOM_EX"
                << std::endl;
      cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255),
                 -1);
    }
    if (event.event_type == MIDDLE) {
      map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50, 50);
    }
    if (event.event_type == CEILING) {
      map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255, 255);
    }
    if (event.event_type == FLOOR) {
      map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0, 0);
    }
  }
}

void CheckPointType(const cv::Mat& map, const Polygon& wall,
                    const PolygonList& obstacles) {
  cv::Mat vis_map = map.clone();
  cv::cvtColor(vis_map, vis_map, cv::COLOR_GRAY2BGR);

  cv::namedWindow("map", cv::WINDOW_NORMAL);
  std::cout
      << "----------------------------CheckPointType-----------------------"
      << std::endl;
  for (const auto& obstacle : obstacles) {
    CheckObstaclePointType(vis_map, obstacle);
    cv::imshow("map", vis_map);
    cv::waitKey(0);
    std::cout << "--------------------------" << std::endl;
  }

  CheckWallPointType(vis_map, wall);
  cv::imshow("map", vis_map);
  cv::waitKey(0);
}

void CheckGeneratedCells(const cv::Mat& map,
                         const std::vector<CellNode>& cell_graph) {
  cv::Mat vis_map = map.clone();
  cv::cvtColor(vis_map, vis_map, cv::COLOR_GRAY2BGR);

  cv::namedWindow("map", cv::WINDOW_NORMAL);
  std::cout << "***********CheckGeneratedCells********************************"
            << std::endl;
  for (const auto& cell : cell_graph) {
    map_visualization::DrawCells(vis_map, cell, cv::Scalar(255, 0, 255));
    cv::imshow("map", vis_map);
    cv::waitKey(0);
  }
}

void CheckPathNodes(const std::deque<std::deque<Point2D>>& path) {
  for (const auto& subpath : path) {
    for (const auto& point : subpath) {
      std::cout << point.x << ", " << point.y << std::endl;
    }
    std::cout << std::endl;
  }
}

/** 测试用例 **/

/** 使用图像数据 **/
void StaticPathPlanningExample1() {
  double meters_per_pix = 0.02;
  double robot_size_in_meters = 0.15;

  int robot_radius = ComputeRobotRadius(meters_per_pix, robot_size_in_meters);

  cv::Mat1b map = map_sdk::ReadMap("../map.png");
  map = map_sdk::PreprocessMap(map);

  std::vector<std::vector<cv::Point>> obstacle_contours;
  std::vector<std::vector<cv::Point>> wall_contours;
  //从原始地图中提取墙和障碍物的轮廓，并根据机器人的半径对轮廓进行
  map_sdk::ExtractContours(map, wall_contours, obstacle_contours, robot_radius);

  Polygon wall = map_sdk::ConstructWall(map, wall_contours.front());
  PolygonList obstacles = map_sdk::ConstructObstacles(map, obstacle_contours);
  //执行分区操作，获取分区结果cell_graph
  std::vector<CellNode> cell_graph = bcd.ConstructCellGraph(
      map, wall_contours, obstacle_contours, wall, obstacles);

  //设定起始点:默认地图中间
  Point2D start = Point2D(map.cols / 2, map.rows / 2);
  //根据起始点和分区结果进行路径规划
  std::deque<std::deque<Point2D>> original_planning_path =
      StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);

  //去除连续重复路径
  std::deque<Point2D> path = FilterTrajectory(original_planning_path);
  //检查路径的连续性和重复性
  CheckPathConsistency(path);
  //弓字规划路径可视化显示
  map_visualization::VisualizeTrajectory(map, path, robot_radius, ROBOT_MODE);

  //将路径分段打印 just test
  // Eigen::Vector2d curr_direction = {0, -1};
  // std::vector<NavigationMessage> messages =
  //     GetNavigationMessage(curr_direction, path, meters_per_pix);
  // PrintMotionCommands(messages);
}

void StaticPathPlanningExample2() {
  int robot_radius = 5;

  cv::Mat1b map = map_sdk::ReadMap("../complicate_map.png");
  map = map_sdk::PreprocessMap(map);

  std::vector<std::vector<cv::Point>> wall_contours;
  std::vector<std::vector<cv::Point>> obstacle_contours;
  map_sdk::ExtractContours(map, wall_contours, obstacle_contours);

  Polygon wall = map_sdk::ConstructWall(map, wall_contours.front());
  PolygonList obstacles = map_sdk::ConstructObstacles(map, obstacle_contours);

  std::vector<CellNode> cell_graph = bcd.ConstructCellGraph(
      map, wall_contours, obstacle_contours, wall, obstacles);

  Point2D start = cell_graph.front().ceiling.front();
  std::deque<std::deque<Point2D>> original_planning_path =
      StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);

  std::deque<Point2D> path = FilterTrajectory(original_planning_path);
  CheckPathConsistency(path);

  int time_interval = 1;
  map_visualization::VisualizeTrajectory(map, path, robot_radius, PATH_MODE,
                                         time_interval);
}

/** 使用构造数据 **/
void StaticPathPlanningExample3() {
  int robot_radius = 5;

  cv::Mat1b map = cv::Mat1b(cv::Size(500, 500), CV_8U);
  map.setTo(255);

  std::vector<std::vector<cv::Point>> contours =
      ConstructHandcraftedContours1();
  cv::fillPoly(map, contours, 0);

  std::vector<std::vector<cv::Point>> obstacle_contours;
  std::vector<std::vector<cv::Point>> wall_contours;
  map_sdk::ExtractContours(map, wall_contours, obstacle_contours);
  map_visualization::VisualizeExtractedContours(map, wall_contours);
  map_visualization::VisualizeExtractedContours(map, obstacle_contours);

  PolygonList obstacles = map_sdk::ConstructObstacles(map, obstacle_contours);
  Polygon wall = map_sdk::ConstructWall(map, wall_contours.front());

  std::vector<CellNode> cell_graph = bcd.ConstructCellGraph(
      map, wall_contours, obstacle_contours, wall, obstacles);
  CheckPointType(map, wall, obstacles);
  CheckGeneratedCells(map, cell_graph);

  Point2D start = cell_graph.front().ceiling.front();
  std::deque<std::deque<Point2D>> original_planning_path =
      StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
  CheckPathNodes(original_planning_path);

  std::deque<Point2D> path = FilterTrajectory(original_planning_path);
  CheckPathConsistency(path);

  int time_interval = 1;
  map_visualization::VisualizeTrajectory(map, path, robot_radius, PATH_MODE,
                                         time_interval);
}

void StaticPathPlanningExample4() {
  int robot_radius = 5;

  cv::Mat1b map = cv::Mat1b(cv::Size(600, 600), CV_8U);
  map.setTo(255);

  std::vector<std::vector<cv::Point>> contours =
      ConstructHandcraftedContours2();
  cv::fillPoly(map, contours, 0);

  std::vector<std::vector<cv::Point>> obstacle_contours;
  std::vector<std::vector<cv::Point>> wall_contours;
  map_sdk::ExtractContours(map, wall_contours, obstacle_contours);
  map_visualization::VisualizeExtractedContours(map, wall_contours);
  map_visualization::VisualizeExtractedContours(map, obstacle_contours);

  PolygonList obstacles = map_sdk::ConstructObstacles(map, obstacle_contours);
  Polygon wall = map_sdk::ConstructWall(map, wall_contours.front());

  cv::Mat3b map_ = cv::Mat3b(map.size());
  map_.setTo(cv::Scalar(0, 0, 0));

  cv::fillPoly(map_, wall_contours, cv::Scalar(255, 255, 255));
  cv::fillPoly(map_, obstacle_contours, cv::Scalar(0, 0, 0));

  std::vector<Event> wall_event_list = bcd.GenerateWallEventList(map_, wall);
  std::vector<Event> obstacle_event_list =
      bcd.GenerateObstacleEventList(map_, obstacles);
  std::deque<std::deque<Event>> slice_list =
      bcd.SliceListGenerator(wall_event_list, obstacle_event_list);
  bcd.CheckSlicelist(slice_list);

  std::vector<CellNode> cell_graph;
  std::vector<int> cell_index_slice;
  std::vector<int> original_cell_index_slice;
  bcd.ExecuteCellDecomposition(cell_graph, cell_index_slice,
                               original_cell_index_slice, slice_list);
  CheckPointType(map, wall, obstacles);
  CheckGeneratedCells(map, cell_graph);

  Point2D start = cell_graph.front().ceiling.front();
  std::deque<std::deque<Point2D>> original_planning_path =
      StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
  CheckPathNodes(original_planning_path);

  std::deque<Point2D> path = FilterTrajectory(original_planning_path);
  CheckPathConsistency(path);

  int time_interval = 1;
  map_visualization::VisualizeTrajectory(map, path, robot_radius, PATH_MODE,
                                         time_interval);
}

void StaticPathPlanningExample5() {
  int robot_radius = 5;

  cv::Mat1b map = cv::Mat1b(cv::Size(600, 600), CV_8U);
  map.setTo(255);

  std::vector<std::vector<cv::Point>> contours =
      ConstructHandcraftedContours3();
  cv::fillPoly(map, contours, 0);

  std::vector<std::vector<cv::Point>> obstacle_contours;
  std::vector<std::vector<cv::Point>> wall_contours;
  map_sdk::ExtractContours(map, wall_contours, obstacle_contours);
  map_visualization::VisualizeExtractedContours(map, wall_contours);
  map_visualization::VisualizeExtractedContours(map, obstacle_contours);

  PolygonList obstacles = map_sdk::ConstructObstacles(map, obstacle_contours);
  Polygon wall = map_sdk::ConstructWall(map, wall_contours.front());

  std::vector<CellNode> cell_graph = bcd.ConstructCellGraph(
      map, wall_contours, obstacle_contours, wall, obstacles);
  CheckPointType(map, wall, obstacles);
  CheckGeneratedCells(map, cell_graph);

  Point2D start = cell_graph.front().ceiling.front();
  std::deque<std::deque<Point2D>> original_planning_path =
      StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
  CheckPathNodes(original_planning_path);

  std::deque<Point2D> path = FilterTrajectory(original_planning_path);
  CheckPathConsistency(path);

  int time_interval = 1;
  map_visualization::VisualizeTrajectory(map, path, robot_radius, PATH_MODE,
                                         time_interval);
}

void StaticPathPlanningExample6() {
  int robot_radius = 25;

  cv::Mat1b map = cv::Mat1b(cv::Size(600, 600), CV_8U);
  map.setTo(0);

  std::vector<std::vector<cv::Point>> contours =
      ConstructHandcraftedContours4();
  std::vector<std::vector<cv::Point>> external_contours = {contours.front()};
  std::vector<std::vector<cv::Point>> inner_contours = {contours.back()};
  cv::fillPoly(map, external_contours, 255);
  cv::fillPoly(map, inner_contours, 0);

  std::vector<std::vector<cv::Point>> obstacle_contours;
  std::vector<std::vector<cv::Point>> wall_contours;
  map_sdk::ExtractContours(map, wall_contours, obstacle_contours);
  // wall_contours and obstacle_contours visualization
  std::cout << "****************************************check contours, "
               "wall_contours...."
            << std::endl;
  map_visualization::VisualizeExtractedContours(map, wall_contours);
  std::cout << "***********************************check contours, "
               "obstacle_contours------"
            << std::endl;
  map_visualization::VisualizeExtractedContours(map, obstacle_contours);

  // cv::imwrite("sssss.png", map);
  PolygonList obstacles = map_sdk::ConstructObstacles(map, obstacle_contours);

  // Debug print
  cv::imshow("ConstructObstacles_map", map);
  cv::waitKey(0);

  Polygon wall = map_sdk::ConstructWall(map, wall_contours.front());

  // Debug print
  cv::imshow("map_sdk::ConstructWall_map", map);
  cv::waitKey(0);

  std::vector<CellNode> cell_graph = bcd.ConstructCellGraph(
      map, wall_contours, obstacle_contours, wall, obstacles);
  CheckPointType(map, wall, obstacles);
  CheckGeneratedCells(map, cell_graph);

  Point2D start = cell_graph.front().ceiling.front();
  std::deque<std::deque<Point2D>> original_planning_path =
      StaticPathPlanning(map, cell_graph, start, robot_radius, true, true);
  CheckPathNodes(original_planning_path);

  std::deque<Point2D> path = FilterTrajectory(original_planning_path);
  CheckPathConsistency(path);

  int time_interval = 1;
  std::cout << "************VisualizeTrajectory**************************"
            << std::endl;
  map_visualization::VisualizeTrajectory(map, path, robot_radius, PATH_MODE,
                                         time_interval);
}

// 未完成
void DynamicPathPlanningExample1() {}

void TestAllExamples() {
  // StaticPathPlanningExample1();

  StaticPathPlanningExample2();

  // StaticPathPlanningExample3();

  // StaticPathPlanningExample4();

  // StaticPathPlanningExample5();

  // StaticPathPlanningExample6();
}

int main() {
  TestAllExamples();

  return 0;
}