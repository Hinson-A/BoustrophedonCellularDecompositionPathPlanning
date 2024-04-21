#include "bcd.h"

// 牛耕分区
std::vector<CellNode> BoustrophedonCellDecomposition::ConstructCellGraph(
    const cv::Mat& original_map,
    const std::vector<std::vector<cv::Point>>& wall_contours,
    const std::vector<std::vector<cv::Point>>& obstacle_contours,
    const Polygon& wall, const PolygonList& obstacles) {
  cv::Mat3b map = cv::Mat3b(original_map.size());
  map.setTo(cv::Scalar(0, 0, 0));

  cv::fillPoly(map, wall_contours, cv::Scalar(255, 255, 255));
  cv::fillPoly(map, obstacle_contours, cv::Scalar(0, 0, 0));

  std::vector<Event> wall_event_list = GenerateWallEventList(map, wall);
  std::vector<Event> obstacle_event_list =
      GenerateObstacleEventList(map, obstacles);
  std::deque<std::deque<Event>> slice_list =
      SliceListGenerator(wall_event_list, obstacle_event_list);

  std::vector<CellNode> cell_graph;
  std::vector<int> cell_index_slice;
  std::vector<int> original_cell_index_slice;
  ExecuteCellDecomposition(cell_graph, cell_index_slice,
                           original_cell_index_slice, slice_list);

  return cell_graph;
}

void BoustrophedonCellDecomposition::ExecuteCellDecomposition(
    std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice,
    std::vector<int>& original_cell_index_slice,
    const std::deque<std::deque<Event>>& slice_list) {
  int curr_cell_idx = INT_MAX;
  int top_cell_idx = INT_MAX;
  int bottom_cell_idx = INT_MAX;

  Point2D c, f;
  int c_index = INT_MAX, f_index = INT_MAX;
  int min_dist = INT_MAX;

  int event_y = INT_MAX;

  bool rewrite = false;

  std::vector<int> sub_cell_index_slices;
  std::deque<Event> curr_slice;

  int cell_counter = 0;

  for (const auto& raw_slice : slice_list) {
    curr_slice = FilterSlice(raw_slice);

    original_cell_index_slice.assign(cell_index_slice.begin(),
                                     cell_index_slice.end());

    for (int j = 0; j < curr_slice.size(); j++) {
      if (curr_slice[j].event_type == INNER_IN_EX) {
        event_y = curr_slice[j].y;
        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k]) ==
                      original_cell_index_slice.end();  // 若为true，则覆盖

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            curr_cell_idx = cell_index_slice[k];
            ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                 Point2D(curr_slice[j].x, curr_slice[j].y), c,
                                 f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k);
              sub_cell_index_slices.clear();
              sub_cell_index_slices = {int(cell_graph.size() - 2),
                                       int(cell_graph.size() - 1)};
              cell_index_slice.insert(cell_index_slice.begin() + k,
                                      sub_cell_index_slices.begin(),
                                      sub_cell_index_slices.end());
            } else {
              cell_index_slice.insert(cell_index_slice.begin() + k + 1,
                                      int(cell_graph.size() - 1));
            }

            curr_slice[j].isUsed = true;

            break;
          }
        }
      }
      if (curr_slice[j].event_type == INNER_OUT_EX) {
        event_y = curr_slice[j].y;
        for (int k = 1; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k - 1]) ==
                      original_cell_index_slice.end();

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k - 1]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            top_cell_idx = cell_index_slice[k - 1];
            bottom_cell_idx = cell_index_slice[k];

            ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx, c,
                                  f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.insert(cell_index_slice.begin() + k - 1,
                                      int(cell_graph.size() - 1));
            } else {
              cell_index_slice.erase(cell_index_slice.begin() + k);
            }

            curr_slice[j].isUsed = true;

            break;
          }
        }
      }

      if (curr_slice[j].event_type == INNER_IN_BOTTOM_EX) {
        event_y = curr_slice[j].y;
        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k]) ==
                      original_cell_index_slice.end();

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            curr_cell_idx = cell_index_slice[k];
            ExecuteOpenOperation(
                cell_graph, curr_cell_idx,
                Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y),  // in top
                Point2D(curr_slice[j].x, curr_slice[j].y),          // in bottom
                c, f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k);
              sub_cell_index_slices.clear();
              sub_cell_index_slices = {int(cell_graph.size() - 2),
                                       int(cell_graph.size() - 1)};
              cell_index_slice.insert(cell_index_slice.begin() + k,
                                      sub_cell_index_slices.begin(),
                                      sub_cell_index_slices.end());
            } else {
              cell_index_slice.insert(cell_index_slice.begin() + k + 1,
                                      int(cell_graph.size() - 1));
            }

            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;

            break;
          }
        }
      }

      if (curr_slice[j].event_type == INNER_OUT_BOTTOM_EX) {
        event_y = curr_slice[j].y;
        for (int k = 1; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k - 1]) ==
                      original_cell_index_slice.end();

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k - 1]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            top_cell_idx = cell_index_slice[k - 1];
            bottom_cell_idx = cell_index_slice[k];
            ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx, c,
                                  f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.insert(cell_index_slice.begin() + k - 1,
                                      int(cell_graph.size() - 1));
            } else {
              cell_index_slice.erase(cell_index_slice.begin() + k);
            }

            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;

            break;
          }
        }
      }

      if (curr_slice[j].event_type == IN_EX) {
        event_y = curr_slice[j].y;

        if (!cell_index_slice.empty()) {
          for (int k = 1; k < cell_index_slice.size(); k++) {
            if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y &&
                event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y) {
              ExecuteInnerOpenOperation(
                  cell_graph,
                  Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
              cell_index_slice.insert(cell_index_slice.begin() + k,
                                      int(cell_graph.size() - 1));
              curr_slice[j].isUsed = true;
              break;
            }
          }
          if (event_y <=
              cell_graph[cell_index_slice.front()].ceiling.back().y) {
            ExecuteInnerOpenOperation(
                cell_graph,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
            cell_index_slice.insert(cell_index_slice.begin(),
                                    int(cell_graph.size() - 1));
            curr_slice[j].isUsed = true;
          }
          if (event_y >= cell_graph[cell_index_slice.back()].floor.back().y) {
            ExecuteInnerOpenOperation(
                cell_graph,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
            cell_index_slice.insert(cell_index_slice.end(),
                                    int(cell_graph.size() - 1));
            curr_slice[j].isUsed = true;
          }

        } else {
          ExecuteInnerOpenOperation(
              cell_graph,
              Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
          cell_index_slice.emplace_back(int(cell_graph.size() - 1));
          curr_slice[j].isUsed = true;
        }
      }

      if (curr_slice[j].event_type == IN_BOTTOM_EX) {
        event_y = curr_slice[j].y;

        if (!cell_index_slice.empty()) {
          for (int k = 1; k < cell_index_slice.size(); k++) {
            if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y &&
                event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y) {
              ExecuteInnerOpenOperation(
                  cell_graph,
                  Point2D(curr_slice[j - 1].x,
                          curr_slice[j - 1].y),  // inner_in_top,
                  Point2D(curr_slice[j].x,
                          curr_slice[j].y));  // inner_in_bottom

              cell_index_slice.insert(cell_index_slice.begin() + k,
                                      int(cell_graph.size() - 1));

              curr_slice[j - 1].isUsed = true;
              curr_slice[j].isUsed = true;

              break;
            }
          }
          if (event_y <=
              cell_graph[cell_index_slice.front()].ceiling.back().y) {
            ExecuteInnerOpenOperation(
                cell_graph,
                Point2D(curr_slice[j - 1].x,
                        curr_slice[j - 1].y),                // inner_in_top,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in_bottom

            cell_index_slice.insert(cell_index_slice.begin(),
                                    int(cell_graph.size() - 1));

            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;
          }
          if (event_y >= cell_graph[cell_index_slice.back()].floor.back().y) {
            ExecuteInnerOpenOperation(
                cell_graph,
                Point2D(curr_slice[j - 1].x,
                        curr_slice[j - 1].y),                // inner_in_top,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in_bottom

            cell_index_slice.insert(cell_index_slice.end(),
                                    int(cell_graph.size() - 1));

            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;
          }
        } else {
          ExecuteInnerOpenOperation(
              cell_graph,
              Point2D(curr_slice[j - 1].x,
                      curr_slice[j - 1].y),                // inner_in_top,
              Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in_bottom

          cell_index_slice.emplace_back(int(cell_graph.size() - 1));

          curr_slice[j - 1].isUsed = true;
          curr_slice[j].isUsed = true;
        }
      }

      if (curr_slice[j].event_type == OUT_EX) {
        event_y = curr_slice[j].y;

        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y <= cell_graph[cell_index_slice[k]].floor.back().y) {
            curr_cell_idx = cell_index_slice[k];
            ExecuteInnerCloseOperation(
                cell_graph, curr_cell_idx,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
            cell_index_slice.erase(cell_index_slice.begin() + k);
            curr_slice[j].isUsed = true;
            break;
          }
        }
      }

      if (curr_slice[j].event_type == OUT_BOTTOM_EX) {
        event_y = curr_slice[j].y;

        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y <= cell_graph[cell_index_slice[k]].floor.back().y) {
            curr_cell_idx = cell_index_slice[k];
            ExecuteInnerCloseOperation(
                cell_graph, curr_cell_idx,
                Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y),
                Point2D(curr_slice[j].x,
                        curr_slice[j].y));  // inner_out_top, inner_out_bottom
            cell_index_slice.erase(cell_index_slice.begin() + k);
            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;
            break;
          }
        }
      }
    }

    for (int j = 0; j < curr_slice.size(); j++) {
      if (curr_slice[j].event_type == IN) {
        event_y = curr_slice[j].y;
        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k]) ==
                      original_cell_index_slice.end();  // 若为true，则覆盖

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            curr_cell_idx = cell_index_slice[k];
            ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                 Point2D(curr_slice[j].x, curr_slice[j].y), c,
                                 f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k);
              sub_cell_index_slices.clear();
              sub_cell_index_slices = {int(cell_graph.size() - 2),
                                       int(cell_graph.size() - 1)};
              cell_index_slice.insert(cell_index_slice.begin() + k,
                                      sub_cell_index_slices.begin(),
                                      sub_cell_index_slices.end());
            } else {
              cell_index_slice.insert(cell_index_slice.begin() + k + 1,
                                      int(cell_graph.size() - 1));
            }

            curr_slice[j].isUsed = true;

            break;
          }
        }
      }
      if (curr_slice[j].event_type == OUT) {
        event_y = curr_slice[j].y;
        for (int k = 1; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k - 1]) ==
                      original_cell_index_slice.end();

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k - 1]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            top_cell_idx = cell_index_slice[k - 1];
            bottom_cell_idx = cell_index_slice[k];

            ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx, c,
                                  f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.insert(cell_index_slice.begin() + k - 1,
                                      int(cell_graph.size() - 1));
            } else {
              cell_index_slice.erase(cell_index_slice.begin() + k);
            }

            curr_slice[j].isUsed = true;

            break;
          }
        }
      }

      if (curr_slice[j].event_type == IN_BOTTOM) {
        event_y = curr_slice[j].y;
        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k]) ==
                      original_cell_index_slice.end();

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            curr_cell_idx = cell_index_slice[k];
            ExecuteOpenOperation(
                cell_graph, curr_cell_idx,
                Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y),  // in top
                Point2D(curr_slice[j].x, curr_slice[j].y),          // in bottom
                c, f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k);
              sub_cell_index_slices.clear();
              sub_cell_index_slices = {int(cell_graph.size() - 2),
                                       int(cell_graph.size() - 1)};
              cell_index_slice.insert(cell_index_slice.begin() + k,
                                      sub_cell_index_slices.begin(),
                                      sub_cell_index_slices.end());
            } else {
              cell_index_slice.insert(cell_index_slice.begin() + k + 1,
                                      int(cell_graph.size() - 1));
            }

            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;

            break;
          }
        }
      }

      if (curr_slice[j].event_type == OUT_BOTTOM) {
        event_y = curr_slice[j].y;
        for (int k = 1; k < cell_index_slice.size(); k++) {
          if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y &&
              event_y < cell_graph[cell_index_slice[k]].floor.back().y) {
            rewrite = std::find(original_cell_index_slice.begin(),
                                original_cell_index_slice.end(),
                                cell_index_slice[k - 1]) ==
                      original_cell_index_slice.end();

            min_dist = INT_MAX;
            for (int m = 0; m < curr_slice.size(); m++) {
              if (abs(curr_slice[m].y -
                      cell_graph[cell_index_slice[k - 1]].ceiling.back().y) <
                  min_dist) {
                min_dist =
                    abs(curr_slice[m].y -
                        cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                c_index = m;
                c = Point2D(curr_slice[m].x, curr_slice[m].y);
              }
            }
            curr_slice[c_index].isUsed = true;

            min_dist = INT_MAX;
            for (int n = 0; n < curr_slice.size(); n++) {
              if (abs(curr_slice[n].y -
                      cell_graph[cell_index_slice[k]].floor.back().y) <
                  min_dist) {
                min_dist = abs(curr_slice[n].y -
                               cell_graph[cell_index_slice[k]].floor.back().y);
                f_index = n;
                f = Point2D(curr_slice[n].x, curr_slice[n].y);
              }
            }
            curr_slice[f_index].isUsed = true;

            top_cell_idx = cell_index_slice[k - 1];
            bottom_cell_idx = cell_index_slice[k];
            ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx, c,
                                  f, rewrite);

            if (!rewrite) {
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.erase(cell_index_slice.begin() + k - 1);
              cell_index_slice.insert(cell_index_slice.begin() + k - 1,
                                      int(cell_graph.size() - 1));
            } else {
              cell_index_slice.erase(cell_index_slice.begin() + k);
            }

            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;

            break;
          }
        }
      }

      if (curr_slice[j].event_type == INNER_IN) {
        event_y = curr_slice[j].y;
        for (int k = 1; k < cell_index_slice.size(); k++) {
          if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y &&
              event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y) {
            ExecuteInnerOpenOperation(
                cell_graph,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
            cell_index_slice.insert(cell_index_slice.begin() + k,
                                    int(cell_graph.size() - 1));
            curr_slice[j].isUsed = true;
            break;
          }
        }
      }

      if (curr_slice[j].event_type == INNER_IN_BOTTOM) {
        event_y = curr_slice[j].y;
        for (int k = 1; k < cell_index_slice.size(); k++) {
          if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y &&
              event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y) {
            ExecuteInnerOpenOperation(
                cell_graph,
                Point2D(curr_slice[j - 1].x,
                        curr_slice[j - 1].y),                // inner_in_top,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in_bottom

            cell_index_slice.insert(cell_index_slice.begin() + k,
                                    int(cell_graph.size() - 1));

            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;

            break;
          }
        }
      }

      if (curr_slice[j].event_type == INNER_OUT) {
        event_y = curr_slice[j].y;
        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y <= cell_graph[cell_index_slice[k]].floor.back().y) {
            curr_cell_idx = cell_index_slice[k];
            ExecuteInnerCloseOperation(
                cell_graph, curr_cell_idx,
                Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
            cell_index_slice.erase(cell_index_slice.begin() + k);
            curr_slice[j].isUsed = true;
            break;
          }
        }
      }

      if (curr_slice[j].event_type == INNER_OUT_BOTTOM) {
        event_y = curr_slice[j].y;
        for (int k = 0; k < cell_index_slice.size(); k++) {
          if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y &&
              event_y <= cell_graph[cell_index_slice[k]].floor.back().y) {
            curr_cell_idx = cell_index_slice[k];
            ExecuteInnerCloseOperation(
                cell_graph, curr_cell_idx,
                Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y),
                Point2D(curr_slice[j].x,
                        curr_slice[j].y));  // inner_out_top, inner_out_bottom
            cell_index_slice.erase(cell_index_slice.begin() + k);
            curr_slice[j - 1].isUsed = true;
            curr_slice[j].isUsed = true;
            break;
          }
        }
      }
    }

    for (int j = 0; j < curr_slice.size(); j++) {
      if (curr_slice[j].event_type == CEILING) {
        cell_counter = CountCells(curr_slice, j);
        curr_cell_idx = cell_index_slice[cell_counter];
        if (!curr_slice[j].isUsed) {
          ExecuteCeilOperation(cell_graph, curr_cell_idx,
                               Point2D(curr_slice[j].x, curr_slice[j].y));
        }
      }
      if (curr_slice[j].event_type == FLOOR) {
        cell_counter = CountCells(curr_slice, j);
        curr_cell_idx = cell_index_slice[cell_counter];
        if (!curr_slice[j].isUsed) {
          ExecuteFloorOperation(cell_graph, curr_cell_idx,
                                Point2D(curr_slice[j].x, curr_slice[j].y));
        }
      }
    }
  }
}

void BoustrophedonCellDecomposition::CheckSlicelist(
    const std::deque<std::deque<Event>>& slice_list) {
  std::deque<Event> slice;
  for (int i = 0; i < slice_list.size(); i++) {
    slice = FilterSlice(slice_list[i]);
    std::cout << "slice " << i << ": ";
    for (const auto& event : slice) {
      EventType type = event.event_type;
      switch (type) {
        case IN:
          std::cout << "IN; ";
          break;
        case IN_TOP:
          std::cout << "IN_TOP; ";
          break;
        case IN_BOTTOM:
          std::cout << "IN_BOTTOM; ";
          break;
        case OUT:
          std::cout << "OUT; ";
          break;
        case OUT_TOP:
          std::cout << "OUT_TOP; ";
          break;
        case OUT_BOTTOM:
          std::cout << "OUT_BOTTOM; ";
          break;
        case INNER_IN:
          std::cout << "INNER_IN; ";
          break;
        case INNER_IN_TOP:
          std::cout << "INNER_IN_TOP; ";
          break;
        case INNER_IN_BOTTOM:
          std::cout << "INNER_IN_BOTTOM; ";
          break;
        case INNER_OUT:
          std::cout << "INNER_OUT; ";
          break;
        case INNER_OUT_TOP:
          std::cout << "INNER_OUT_TOP; ";
          break;
        case INNER_OUT_BOTTOM:
          std::cout << "INNER_OUT_BOTTOM; ";
          break;
        case IN_EX:
          std::cout << "IN_EX; ";
          break;
        case IN_TOP_EX:
          std::cout << "IN_TOP_EX; ";
          break;
        case IN_BOTTOM_EX:
          std::cout << "IN_BOTTOM_EX; ";
          break;
        case OUT_EX:
          std::cout << "OUT_EX; ";
          break;
        case OUT_TOP_EX:
          std::cout << "OUT_TOP_EX; ";
          break;
        case OUT_BOTTOM_EX:
          std::cout << "OUT_BOTTOM_EX; ";
          break;
        case INNER_IN_EX:
          std::cout << "INNER_IN_EX; ";
          break;
        case INNER_IN_TOP_EX:
          std::cout << "INNER_IN_TOP_EX; ";
          break;
        case INNER_IN_BOTTOM_EX:
          std::cout << "INNER_IN_BOTTOM_EX; ";
          break;
        case INNER_OUT_EX:
          std::cout << "INNER_OUT_EX; ";
          break;
        case INNER_OUT_TOP_EX:
          std::cout << "INNER_OUT_TOP_EX; ";
          break;
        case INNER_OUT_BOTTOM_EX:
          std::cout << "INNER_OUT_BOTTOM_EX; ";
          break;
        case MIDDLE:
          std::cout << "MIDDLE; ";
          break;
        case CEILING:
          std::cout << "CEILING; ";
          break;
        case FLOOR:
          std::cout << "FLOOR; ";
          break;
        case UNALLOCATED:
          std::cout << "UNALLOCATED; ";
          break;
      }
    }
    std::cout << std::endl;
  }
}

int BoustrophedonCellDecomposition::CountCells(const std::deque<Event>& slice,
                                               int curr_idx) {
  int cell_num = 0;
  for (int i = 0; i < curr_idx; i++) {
    if ((slice[i].event_type == IN) || (slice[i].event_type == IN_TOP) ||
        (slice[i].event_type == INNER_IN) ||
        (slice[i].event_type == INNER_IN_BOTTOM) ||
        (slice[i].event_type == FLOOR) ||
        (slice[i].event_type == IN_BOTTOM_EX) ||
        (slice[i].event_type == INNER_IN_EX) ||
        (slice[i].event_type == INNER_IN_TOP_EX)) {
      cell_num++;
    }
  }
  return cell_num;
}

std::deque<Event> BoustrophedonCellDecomposition::FilterSlice(
    const std::deque<Event>& slice) {
  std::deque<Event> filtered_slice;

  for (auto event : slice) {
    if (event.event_type != MIDDLE && event.event_type != UNALLOCATED) {
      filtered_slice.emplace_back(event);
    }
  }
  return filtered_slice;
}

std::vector<Event> BoustrophedonCellDecomposition::InitializeEventList(
    const Polygon& polygon, int polygon_index) {
  std::vector<Event> event_list;

  for (const auto& point : polygon) {
    event_list.emplace_back(Event(polygon_index, point.x, point.y));
  }

  return event_list;
}

//为障碍物事件列表中的每个事件分配具体的事件类型
void BoustrophedonCellDecomposition::AllocateObstacleEventType(
    const cv::Mat& map, std::vector<Event>& event_list) {
  int index_offset;
  std::deque<int> in_out_index_list;  // 只存放各种in和out的index

  int N = event_list.size();

  // determine in and out and middle
  for (int i = 0; i < N; i++) {
    if (event_list[i].x < event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x < event_list[((i + 1) % N + N) % N].x) {
      event_list[i].event_type = IN;
      in_out_index_list.emplace_back(i);
    }
    if (event_list[i].x < event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_TOP;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x < event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_TOP;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x < event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_BOTTOM;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x < event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_BOTTOM;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x) {
      event_list[i].event_type = MIDDLE;
    }

    if (event_list[i].x > event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x > event_list[((i + 1) % N + N) % N].x) {
      event_list[i].event_type = OUT;
      in_out_index_list.emplace_back(i);
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x > event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_TOP;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x > event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_TOP;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x > event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_BOTTOM;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x > event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_BOTTOM;
        in_out_index_list.emplace_back(i);
      }
    }
  }

  // determine inner
  Point2D neighbor_point;
  int temp_index;

  for (auto in_out_index : in_out_index_list) {
    if (event_list[in_out_index].event_type == OUT) {
      neighbor_point =
          Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
          cv::Vec3b(0, 0, 0)) {
        event_list[in_out_index].event_type = INNER_OUT;
      }
    }

    if (event_list[in_out_index].event_type == OUT_TOP) {
      neighbor_point =
          Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
          cv::Vec3b(0, 0, 0)) {
        event_list[in_out_index].event_type = INNER_OUT_TOP;
      }
    }

    if (event_list[in_out_index].event_type == OUT_BOTTOM) {
      neighbor_point =
          Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
          cv::Vec3b(0, 0, 0)) {
        event_list[in_out_index].event_type = INNER_OUT_BOTTOM;
      }
    }

    if (event_list[in_out_index].event_type == IN) {
      neighbor_point =
          Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
          cv::Vec3b(0, 0, 0)) {
        event_list[in_out_index].event_type = INNER_IN;
      }
    }

    if (event_list[in_out_index].event_type == IN_TOP) {
      neighbor_point =
          Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
          cv::Vec3b(0, 0, 0)) {
        event_list[in_out_index].event_type = INNER_IN_TOP;
      }
    }

    if (event_list[in_out_index].event_type == IN_BOTTOM) {
      neighbor_point =
          Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
          cv::Vec3b(0, 0, 0)) {
        event_list[in_out_index].event_type = INNER_IN_BOTTOM;
      }
    }
  }

  // determine floor and ceiling
  std::deque<int> ceiling_floor_index_list;

  for (int i = 0; i < in_out_index_list.size(); i++) {
    if ((event_list[in_out_index_list[0]].event_type == OUT ||
         event_list[in_out_index_list[0]].event_type == OUT_TOP ||
         event_list[in_out_index_list[0]].event_type == OUT_BOTTOM ||
         event_list[in_out_index_list[0]].event_type == INNER_OUT ||
         event_list[in_out_index_list[0]].event_type == INNER_OUT_TOP ||
         event_list[in_out_index_list[0]].event_type == INNER_OUT_BOTTOM) &&
        (event_list[in_out_index_list[1]].event_type == IN ||
         event_list[in_out_index_list[1]].event_type == IN_TOP ||
         event_list[in_out_index_list[1]].event_type == IN_BOTTOM ||
         event_list[in_out_index_list[1]].event_type == INNER_IN ||
         event_list[in_out_index_list[1]].event_type == INNER_IN_TOP ||
         event_list[in_out_index_list[1]].event_type == INNER_IN_BOTTOM)) {
      if (in_out_index_list[0] < in_out_index_list[1]) {
        for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = FLOOR;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
      } else {
        for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = FLOOR;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
        for (int k = 0; k < in_out_index_list[1]; k++) {
          if (event_list[k].event_type != MIDDLE) {
            event_list[k].event_type = FLOOR;
            ceiling_floor_index_list.emplace_back(k);
          }
        }
      }
    }

    if ((event_list[in_out_index_list[0]].event_type == IN ||
         event_list[in_out_index_list[0]].event_type == IN_TOP ||
         event_list[in_out_index_list[0]].event_type == IN_BOTTOM ||
         event_list[in_out_index_list[0]].event_type == INNER_IN ||
         event_list[in_out_index_list[0]].event_type == INNER_IN_TOP ||
         event_list[in_out_index_list[0]].event_type == INNER_IN_BOTTOM) &&
        (event_list[in_out_index_list[1]].event_type == OUT ||
         event_list[in_out_index_list[1]].event_type == OUT_TOP ||
         event_list[in_out_index_list[1]].event_type == OUT_BOTTOM ||
         event_list[in_out_index_list[1]].event_type == INNER_OUT ||
         event_list[in_out_index_list[1]].event_type == INNER_OUT_TOP ||
         event_list[in_out_index_list[1]].event_type == INNER_OUT_BOTTOM)) {
      if (in_out_index_list[0] < in_out_index_list[1]) {
        for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = CEILING;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
      } else {
        for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = CEILING;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
        for (int k = 0; k < in_out_index_list[1]; k++) {
          if (event_list[k].event_type != MIDDLE) {
            event_list[k].event_type = CEILING;
            ceiling_floor_index_list.emplace_back(k);
          }
        }
      }
    }

    temp_index = in_out_index_list.front();
    in_out_index_list.pop_front();
    in_out_index_list.emplace_back(temp_index);
  }

  // filter ceiling and floor
  for (int i = 0; i < ceiling_floor_index_list.size() - 1; i++) {
    if (event_list[ceiling_floor_index_list[i]].event_type == CEILING &&
        event_list[ceiling_floor_index_list[i + 1]].event_type == CEILING &&
        event_list[ceiling_floor_index_list[i]].x ==
            event_list[ceiling_floor_index_list[i + 1]].x) {
      if (event_list[ceiling_floor_index_list[i]].y >
          event_list[ceiling_floor_index_list[i + 1]].y) {
        event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
      } else {
        event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
      }
    }
    if (event_list[ceiling_floor_index_list[i]].event_type == FLOOR &&
        event_list[ceiling_floor_index_list[i + 1]].event_type == FLOOR &&
        event_list[ceiling_floor_index_list[i]].x ==
            event_list[ceiling_floor_index_list[i + 1]].x) {
      if (event_list[ceiling_floor_index_list[i]].y <
          event_list[ceiling_floor_index_list[i + 1]].y) {
        event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
      } else {
        event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
      }
    }
  }
  if (event_list[ceiling_floor_index_list.back()].event_type == CEILING &&
      event_list[ceiling_floor_index_list.front()].event_type == CEILING &&
      event_list[ceiling_floor_index_list.back()].x ==
          event_list[ceiling_floor_index_list.front()].x) {
    if (event_list[ceiling_floor_index_list.back()].y >
        event_list[ceiling_floor_index_list.front()].y) {
      event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
    } else {
      event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
    }
  }
  if (event_list[ceiling_floor_index_list.back()].event_type == FLOOR &&
      event_list[ceiling_floor_index_list.front()].event_type == FLOOR &&
      event_list[ceiling_floor_index_list.back()].x ==
          event_list[ceiling_floor_index_list.front()].x) {
    if (event_list[ceiling_floor_index_list.back()].y <
        event_list[ceiling_floor_index_list.front()].y) {
      event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
    } else {
      event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
    }
  }
}

void BoustrophedonCellDecomposition::AllocateWallEventType(
    const cv::Mat& map, std::vector<Event>& event_list) {
  int index_offset;
  std::deque<int> in_out_index_list;  // 只存放各种in和out的index

  int N = event_list.size();

  // determine in and out and middle
  for (int i = 0; i < N; i++) {
    if (event_list[i].x < event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x < event_list[((i + 1) % N + N) % N].x) {
      event_list[i].event_type = IN_EX;
      in_out_index_list.emplace_back(i);
    }
    if (event_list[i].x < event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_TOP_EX;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x < event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_TOP_EX;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x < event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_BOTTOM_EX;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x < event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = IN_BOTTOM_EX;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x) {
      event_list[i].event_type = MIDDLE;
    }

    if (event_list[i].x > event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x > event_list[((i + 1) % N + N) % N].x) {
      event_list[i].event_type = OUT_EX;
      in_out_index_list.emplace_back(i);
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x > event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_TOP_EX;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x > event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y < event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y < event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_TOP_EX;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x == event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x > event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i - 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i - index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i - index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_BOTTOM_EX;
        in_out_index_list.emplace_back(i);
      }
    }

    if (event_list[i].x > event_list[((i - 1) % N + N) % N].x &&
        event_list[i].x == event_list[((i + 1) % N + N) % N].x &&
        event_list[i].y > event_list[((i + 1) % N + N) % N].y) {
      index_offset = 2;
      while (event_list[i].x ==
             event_list[((i + index_offset) % N + N) % N].x) {
        index_offset++;
      }
      if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x &&
          event_list[i].y > event_list[((i + index_offset) % N + N) % N].y) {
        event_list[i].event_type = OUT_BOTTOM_EX;
        in_out_index_list.emplace_back(i);
      }
    }
  }

  // determine inner
  Point2D neighbor_point;
  int temp_index;
  for (auto in_out_index : in_out_index_list) {
    if (event_list[in_out_index].event_type == OUT_EX) {
      neighbor_point =
          Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
              cv::Vec3b(255, 255, 255) &&
          neighbor_point.x < map.cols) {
        event_list[in_out_index].event_type = INNER_OUT_EX;
      }
    }

    if (event_list[in_out_index].event_type == OUT_TOP_EX) {
      neighbor_point =
          Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
              cv::Vec3b(255, 255, 255) &&
          neighbor_point.x < map.cols) {
        event_list[in_out_index].event_type = INNER_OUT_TOP_EX;
      }
    }

    if (event_list[in_out_index].event_type == OUT_BOTTOM_EX) {
      neighbor_point =
          Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
              cv::Vec3b(255, 255, 255) &&
          neighbor_point.x < map.cols) {
        event_list[in_out_index].event_type = INNER_OUT_BOTTOM_EX;
      }
    }

    if (event_list[in_out_index].event_type == IN_EX) {
      neighbor_point =
          Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
              cv::Vec3b(255, 255, 255) &&
          neighbor_point.x >= 0) {
        event_list[in_out_index].event_type = INNER_IN_EX;
      }
    }

    if (event_list[in_out_index].event_type == IN_TOP_EX) {
      neighbor_point =
          Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
              cv::Vec3b(255, 255, 255) &&
          neighbor_point.x >= 0) {
        event_list[in_out_index].event_type = INNER_IN_TOP_EX;
      }
    }

    if (event_list[in_out_index].event_type == IN_BOTTOM_EX) {
      neighbor_point =
          Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
      if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) ==
              cv::Vec3b(255, 255, 255) &&
          neighbor_point.x >= 0) {
        event_list[in_out_index].event_type = INNER_IN_BOTTOM_EX;
      }
    }
  }

  // determine floor and ceiling
  std::deque<int> ceiling_floor_index_list;

  for (int i = 0; i < in_out_index_list.size(); i++) {
    if ((event_list[in_out_index_list[0]].event_type == OUT_EX ||
         event_list[in_out_index_list[0]].event_type == OUT_TOP_EX ||
         event_list[in_out_index_list[0]].event_type == OUT_BOTTOM_EX ||
         event_list[in_out_index_list[0]].event_type == INNER_OUT_EX ||
         event_list[in_out_index_list[0]].event_type == INNER_OUT_TOP_EX ||
         event_list[in_out_index_list[0]].event_type == INNER_OUT_BOTTOM_EX) &&
        (event_list[in_out_index_list[1]].event_type == IN_EX ||
         event_list[in_out_index_list[1]].event_type == IN_TOP_EX ||
         event_list[in_out_index_list[1]].event_type == IN_BOTTOM_EX ||
         event_list[in_out_index_list[1]].event_type == INNER_IN_EX ||
         event_list[in_out_index_list[1]].event_type == INNER_IN_TOP_EX ||
         event_list[in_out_index_list[1]].event_type == INNER_IN_BOTTOM_EX)) {
      if (in_out_index_list[0] < in_out_index_list[1]) {
        for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = CEILING;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
      } else {
        for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = CEILING;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
        for (int k = 0; k < in_out_index_list[1]; k++) {
          if (event_list[k].event_type != MIDDLE) {
            event_list[k].event_type = CEILING;
            ceiling_floor_index_list.emplace_back(k);
          }
        }
      }
    }

    if ((event_list[in_out_index_list[0]].event_type == IN_EX ||
         event_list[in_out_index_list[0]].event_type == IN_TOP_EX ||
         event_list[in_out_index_list[0]].event_type == IN_BOTTOM_EX ||
         event_list[in_out_index_list[0]].event_type == INNER_IN_EX ||
         event_list[in_out_index_list[0]].event_type == INNER_IN_TOP_EX ||
         event_list[in_out_index_list[0]].event_type == INNER_IN_BOTTOM_EX) &&
        (event_list[in_out_index_list[1]].event_type == OUT_EX ||
         event_list[in_out_index_list[1]].event_type == OUT_TOP_EX ||
         event_list[in_out_index_list[1]].event_type == OUT_BOTTOM_EX ||
         event_list[in_out_index_list[1]].event_type == INNER_OUT_EX ||
         event_list[in_out_index_list[1]].event_type == INNER_OUT_TOP_EX ||
         event_list[in_out_index_list[1]].event_type == INNER_OUT_BOTTOM_EX)) {
      if (in_out_index_list[0] < in_out_index_list[1]) {
        for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = FLOOR;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
      } else {
        for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++) {
          if (event_list[j].event_type != MIDDLE) {
            event_list[j].event_type = FLOOR;
            ceiling_floor_index_list.emplace_back(j);
          }
        }
        for (int k = 0; k < in_out_index_list[1]; k++) {
          if (event_list[k].event_type != MIDDLE) {
            event_list[k].event_type = FLOOR;
            ceiling_floor_index_list.emplace_back(k);
          }
        }
      }
    }

    temp_index = in_out_index_list.front();
    in_out_index_list.pop_front();
    in_out_index_list.emplace_back(temp_index);
  }

  // filter ceiling and floor
  for (int i = 0; i < ceiling_floor_index_list.size() - 1; i++) {
    if (event_list[ceiling_floor_index_list[i]].event_type == CEILING &&
        event_list[ceiling_floor_index_list[i + 1]].event_type == CEILING &&
        event_list[ceiling_floor_index_list[i]].x ==
            event_list[ceiling_floor_index_list[i + 1]].x) {
      if (event_list[ceiling_floor_index_list[i]].y >
          event_list[ceiling_floor_index_list[i + 1]].y) {
        event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
      } else {
        event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
      }
    }
    if (event_list[ceiling_floor_index_list[i]].event_type == FLOOR &&
        event_list[ceiling_floor_index_list[i + 1]].event_type == FLOOR &&
        event_list[ceiling_floor_index_list[i]].x ==
            event_list[ceiling_floor_index_list[i + 1]].x) {
      if (event_list[ceiling_floor_index_list[i]].y <
          event_list[ceiling_floor_index_list[i + 1]].y) {
        event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
      } else {
        event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
      }
    }
  }
  if (event_list[ceiling_floor_index_list.back()].event_type == CEILING &&
      event_list[ceiling_floor_index_list.front()].event_type == CEILING &&
      event_list[ceiling_floor_index_list.back()].x ==
          event_list[ceiling_floor_index_list.front()].x) {
    if (event_list[ceiling_floor_index_list.back()].y >
        event_list[ceiling_floor_index_list.front()].y) {
      event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
    } else {
      event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
    }
  }
  if (event_list[ceiling_floor_index_list.back()].event_type == FLOOR &&
      event_list[ceiling_floor_index_list.front()].event_type == FLOOR &&
      event_list[ceiling_floor_index_list.back()].x ==
          event_list[ceiling_floor_index_list.front()].x) {
    if (event_list[ceiling_floor_index_list.back()].y <
        event_list[ceiling_floor_index_list.front()].y) {
      event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
    } else {
      event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
    }
  }
}

//生成墙事件
std::vector<Event> BoustrophedonCellDecomposition::GenerateObstacleEventList(
    const cv::Mat& map, const PolygonList& polygons) {
  std::vector<Event> event_list;
  std::vector<Event> event_sublist;

  for (int i = 0; i < polygons.size(); i++) {
    event_sublist = InitializeEventList(polygons[i], i);
    AllocateObstacleEventType(map, event_sublist);
    event_list.insert(event_list.end(), event_sublist.begin(),
                      event_sublist.end());
    event_sublist.clear();
  }

  std::sort(event_list.begin(), event_list.end());

  return event_list;
}

//生成障碍物事件
std::vector<Event> BoustrophedonCellDecomposition::GenerateWallEventList(
    const cv::Mat& map, const Polygon& external_contour) {
  std::vector<Event> event_list;

  event_list = InitializeEventList(external_contour, INT_MAX);
  AllocateWallEventType(map, event_list);
  std::sort(event_list.begin(), event_list.end());

  return event_list;
}

//根据障碍物和墙事件列表生成切片列表
std::deque<std::deque<Event>>
BoustrophedonCellDecomposition::SliceListGenerator(
    const std::vector<Event>& wall_event_list,
    const std::vector<Event>& obstacle_event_list) {
  std::vector<Event> event_list;
  event_list.insert(event_list.end(), obstacle_event_list.begin(),
                    obstacle_event_list.end());
  event_list.insert(event_list.end(), wall_event_list.begin(),
                    wall_event_list.end());
  std::sort(event_list.begin(), event_list.end());

  std::deque<std::deque<Event>> slice_list;
  std::deque<Event> slice;
  int x = event_list.front().x;

  for (auto event : event_list) {
    if (event.x != x) {
      slice_list.emplace_back(slice);

      x = event.x;
      slice.clear();
      slice.emplace_back(event);
    } else {
      slice.emplace_back(event);
    }
  }
  slice_list.emplace_back(slice);

  return slice_list;
}

//在单元格图中执行开放操作，用于创建新的单元格或修改现有单元格的边界
void BoustrophedonCellDecomposition::ExecuteOpenOperation(
    std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in, Point2D c,
    Point2D f, bool rewrite) {
  CellNode top_cell, bottom_cell;

  top_cell.ceiling.emplace_back(c);
  top_cell.floor.emplace_back(in);

  bottom_cell.ceiling.emplace_back(in);
  bottom_cell.floor.emplace_back(f);

  if (!rewrite) {
    int top_cell_index = cell_graph.size();
    int bottom_cell_index = cell_graph.size() + 1;

    top_cell.cellIndex = top_cell_index;
    bottom_cell.cellIndex = bottom_cell_index;
    cell_graph.emplace_back(top_cell);
    cell_graph.emplace_back(bottom_cell);

    cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
    cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

    cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
    cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
  } else {
    cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(),
                                             top_cell.ceiling.end());
    cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(),
                                           top_cell.floor.end());

    int bottom_cell_index = cell_graph.size();
    bottom_cell.cellIndex = bottom_cell_index;
    cell_graph.emplace_back(bottom_cell);

    cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()]
        .neighbor_indices.emplace_back(bottom_cell_index);
    cell_graph[bottom_cell_index].neighbor_indices.emplace_back(
        cell_graph[curr_cell_idx].neighbor_indices.back());
  }
}

//单元格图中执行关闭操作，用于完成单元格的边界
void BoustrophedonCellDecomposition::ExecuteCloseOperation(
    std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx,
    Point2D c, Point2D f, bool rewrite) {
  CellNode new_cell;

  new_cell.ceiling.emplace_back(c);
  new_cell.floor.emplace_back(f);

  if (!rewrite) {
    int new_cell_idx = cell_graph.size();
    new_cell.cellIndex = new_cell_idx;

    cell_graph.emplace_back(new_cell);

    cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
    cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

    cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
    cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);
  } else {
    cell_graph[top_cell_idx].ceiling.assign(new_cell.ceiling.begin(),
                                            new_cell.ceiling.end());
    cell_graph[top_cell_idx].floor.assign(new_cell.floor.begin(),
                                          new_cell.floor.end());

    cell_graph[top_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);
    cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
  }
}

void BoustrophedonCellDecomposition::ExecuteCeilOperation(
    std::vector<CellNode>& cell_graph, int curr_cell_idx,
    const Point2D& ceil_point) {
  cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void BoustrophedonCellDecomposition::ExecuteFloorOperation(
    std::vector<CellNode>& cell_graph, int curr_cell_idx,
    const Point2D& floor_point) {
  cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}

void BoustrophedonCellDecomposition::ExecuteOpenOperation(
    std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in_top,
    Point2D in_bottom, Point2D c, Point2D f, bool rewrite) {
  CellNode top_cell, bottom_cell;

  top_cell.ceiling.emplace_back(c);
  top_cell.floor.emplace_back(in_top);

  bottom_cell.ceiling.emplace_back(in_bottom);
  bottom_cell.floor.emplace_back(f);

  if (!rewrite) {
    int top_cell_index = cell_graph.size();
    int bottom_cell_index = cell_graph.size() + 1;

    top_cell.cellIndex = top_cell_index;
    bottom_cell.cellIndex = bottom_cell_index;
    cell_graph.emplace_back(top_cell);
    cell_graph.emplace_back(bottom_cell);

    cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
    cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

    cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
    cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
  } else {
    cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(),
                                             top_cell.ceiling.end());
    cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(),
                                           top_cell.floor.end());

    int bottom_cell_index = cell_graph.size();
    bottom_cell.cellIndex = bottom_cell_index;
    cell_graph.emplace_back(bottom_cell);

    cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()]
        .neighbor_indices.emplace_back(bottom_cell_index);
    cell_graph[bottom_cell_index].neighbor_indices.emplace_back(
        cell_graph[curr_cell_idx].neighbor_indices.back());
  }
}

void BoustrophedonCellDecomposition::ExecuteInnerOpenOperation(
    std::vector<CellNode>& cell_graph, Point2D inner_in) {
  CellNode new_cell;

  new_cell.ceiling.emplace_back(inner_in);
  new_cell.floor.emplace_back(inner_in);

  int new_cell_index = cell_graph.size();

  new_cell.cellIndex = new_cell_index;
  cell_graph.emplace_back(new_cell);
}

void BoustrophedonCellDecomposition::ExecuteInnerOpenOperation(
    std::vector<CellNode>& cell_graph, Point2D inner_in_top,
    Point2D inner_in_bottom) {
  CellNode new_cell;

  new_cell.ceiling.emplace_back(inner_in_top);
  new_cell.floor.emplace_back(inner_in_bottom);

  int new_cell_index = cell_graph.size();

  new_cell.cellIndex = new_cell_index;
  cell_graph.emplace_back(new_cell);
}

void BoustrophedonCellDecomposition::ExecuteInnerCloseOperation(
    std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out) {
  cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out);
  cell_graph[curr_cell_idx].floor.emplace_back(inner_out);
}

void BoustrophedonCellDecomposition::ExecuteInnerCloseOperation(
    std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out_top,
    Point2D inner_out_bottom) {
  cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out_top);
  cell_graph[curr_cell_idx].floor.emplace_back(inner_out_bottom);
}
