#include "visualization.h"
namespace map_visualization {

//可视化轮廓: 在地图上绘制提取的轮廓，并逐个显示轮廓
void VisualizeExtractedContours(
    const cv::Mat& map, const std::vector<std::vector<cv::Point>>& contours) {
  cv::namedWindow("VisualizeExtractedContours_map", cv::WINDOW_NORMAL);

  cv::Mat3b canvas = cv::Mat3b(map.size(), CV_8U);
  canvas.setTo(cv::Scalar(0, 0, 0));  //黑色背景

  for (int i = 0; i <= contours.size() - 1; i++) {
    //轮廓线为蓝色
    cv::drawContours(canvas, contours, i, cv::Scalar(255, 0, 0));
    cv::imshow("VisualizeExtractedContours_map", canvas);
    cv::waitKey(0);
  }
}

void InitializeColorMap(std::deque<cv::Scalar>& JetColorMap, int repeat_times) {
  for (int i = 0; i <= 255; i++) {
    for (int j = 0; j < repeat_times; j++) {
      JetColorMap.emplace_back(cv::Scalar(0, i, 255));
    }
  }

  for (int i = 254; i >= 0; i--) {
    for (int j = 0; j < repeat_times; j++) {
      JetColorMap.emplace_back(cv::Scalar(0, 255, i));
    }
  }

  for (int i = 1; i <= 255; i++) {
    for (int j = 0; j < repeat_times; j++) {
      JetColorMap.emplace_back(cv::Scalar(i, 255, 0));
    }
  }

  for (int i = 254; i >= 0; i--) {
    for (int j = 0; j < repeat_times; j++) {
      JetColorMap.emplace_back(cv::Scalar(255, i, 0));
    }
  }

  for (int i = 1; i <= 255; i++) {
    for (int j = 0; j < repeat_times; j++) {
      JetColorMap.emplace_back(cv::Scalar(255, 0, i));
    }
  }

  for (int i = 254; i >= 1; i--) {
    for (int j = 0; j < repeat_times; j++) {
      JetColorMap.emplace_back(cv::Scalar(i, 0, 255));
    }
  }
}


void UpdateColorMap(std::deque<cv::Scalar>& JetColorMap) {
  cv::Scalar color = JetColorMap.front();
  JetColorMap.pop_front();
  JetColorMap.emplace_back(color);
}

void VisualizeTrajectory(const cv::Mat& original_map,
                         const std::deque<Point2D>& path, int robot_radius,
                         int vis_mode, int time_interval,
                         int colors) {
  cv::Mat3b vis_map;
  cv::cvtColor(original_map, vis_map, cv::COLOR_GRAY2BGR);

  cv::namedWindow("map", cv::WINDOW_NORMAL);

  std::deque<cv::Scalar> JetColorMap;
  int color_repeated_times = path.size() / colors + 1;
  InitializeColorMap(JetColorMap, color_repeated_times);

  switch (vis_mode) {
    case PATH_MODE:
      vis_map.at<cv::Vec3b>(path.front().y, path.front().x) = cv::Vec3b(
          uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
          uchar(JetColorMap.front()[2]));
      cv::imshow("map", vis_map);
      cv::waitKey(0);

      for (const auto& position : path) {
        vis_map.at<cv::Vec3b>(position.y, position.x) = cv::Vec3b(
            uchar(JetColorMap.front()[0]), uchar(JetColorMap.front()[1]),
            uchar(JetColorMap.front()[2]));
        UpdateColorMap(JetColorMap);
        cv::imshow("map", vis_map);
        cv::waitKey(time_interval);
      }
      break;
    case ROBOT_MODE:
      cv::circle(vis_map, cv::Point(path.front().x, path.front().y),
                 robot_radius, cv::Scalar(255, 204, 153), -1);
      cv::imshow("map", vis_map);
      cv::waitKey(0);

      for (const auto& position : path) {
        cv::circle(vis_map, cv::Point(position.x, position.y), robot_radius,
                   cv::Scalar(255, 204, 153), -1);
        cv::imshow("map", vis_map);
        cv::waitKey(time_interval);

        cv::circle(vis_map, cv::Point(position.x, position.y), robot_radius,
                   cv::Scalar(255, 229, 204), -1);
      }
      break;
    default:
      break;
  }

  cv::waitKey(0);
}

void DrawCells(cv::Mat& map, const CellNode& cell, cv::Scalar color) {
  std::cout << "cell " << cell.cellIndex << ": " << std::endl;
  std::cout << "cell's ceiling points: " << cell.ceiling.size() << std::endl;
  std::cout << "cell's floor points: " << cell.floor.size() << std::endl;

  for (const auto& ceiling_point : cell.ceiling) {
    map.at<cv::Vec3b>(ceiling_point.y, ceiling_point.x) =
        cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
  }

  for (const auto& floor_point : cell.floor) {
    map.at<cv::Vec3b>(floor_point.y, floor_point.x) =
        cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
  }

  cv::line(map, cv::Point(cell.ceiling.front().x, cell.ceiling.front().y),
           cv::Point(cell.floor.front().x, cell.floor.front().y), color);
  cv::line(map, cv::Point(cell.ceiling.back().x, cell.ceiling.back().y),
           cv::Point(cell.floor.back().x, cell.floor.back().y), color);
}

}  // namespace map_visualization