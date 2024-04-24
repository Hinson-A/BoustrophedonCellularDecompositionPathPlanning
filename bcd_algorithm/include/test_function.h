// test_functions.h

#ifndef TEST_FUNCTIONS_H_
#define TEST_FUNCTIONS_H_

#include <deque>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
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

/** 静态地图路径规划测试多边形 **/
std::vector<std::vector<cv::Point>> ConstructHandcraftedContours1();
std::vector<std::vector<cv::Point>> ConstructHandcraftedContours2();
std::vector<std::vector<cv::Point>> ConstructHandcraftedContours3();
std::vector<std::vector<cv::Point>> ConstructHandcraftedContours4();
std::vector<std::vector<cv::Point>> ConstructHandcraftedContours5();

int ComputeRobotRadius(const double& meters_per_pix,
                       const double& robot_size_in_meters);

// tets demo
void StaticPathPlanningExample1();

#endif  // TEST_FUNCTIONS_H_