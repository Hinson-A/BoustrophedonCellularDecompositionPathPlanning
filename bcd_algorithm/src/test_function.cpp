#include "test_function.h"

/*---------------------Start 地图数据构造----------------------*/
std::vector<std::vector<cv::Point>> ConstructHandcraftedContours1() {
  std::vector<cv::Point> handcrafted_polygon_1_1 = {
      cv::Point(200, 300), cv::Point(300, 200), cv::Point(200, 100),
      cv::Point(100, 200)};
  std::vector<cv::Point> handcrafted_polygon_1_2 = {
      cv::Point(300, 350), cv::Point(350, 300), cv::Point(300, 250),
      cv::Point(250, 300)};
  std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_1_1,
                                                  handcrafted_polygon_1_2};
  return contours;
}

std::vector<std::vector<cv::Point>> ConstructHandcraftedContours2() {
  std::vector<cv::Point> handcrafted_polygon_2 = {
      cv::Point(125, 125), cv::Point(125, 175), cv::Point(225, 175),
      cv::Point(225, 225), cv::Point(175, 250), cv::Point(225, 300),
      cv::Point(125, 325), cv::Point(125, 375), cv::Point(375, 375),
      cv::Point(375, 325), cv::Point(275, 325), cv::Point(275, 275),
      cv::Point(325, 250), cv::Point(275, 200), cv::Point(375, 175),
      cv::Point(375, 125)};
  std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_2};
  return contours;
}

std::vector<std::vector<cv::Point>> ConstructHandcraftedContours3() {
  std::vector<cv::Point> handcrafted_polygon_3 = {
      cv::Point(100, 100), cv::Point(100, 500), cv::Point(150, 500),
      cv::Point(150, 150), cv::Point(450, 150), cv::Point(450, 300),
      cv::Point(300, 300), cv::Point(300, 250), cv::Point(350, 250),
      cv::Point(350, 200), cv::Point(250, 200), cv::Point(250, 350),
      cv::Point(500, 350), cv::Point(500, 100)};
  std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_3};
  return contours;
}

std::vector<std::vector<cv::Point>> ConstructHandcraftedContours4() {
  std::vector<cv::Point> handcrafted_polygon_4_1 = {
      cv::Point(20, 20),   cv::Point(20, 200),  cv::Point(100, 200),
      cv::Point(100, 399), cv::Point(20, 399),  cv::Point(20, 579),
      cv::Point(200, 579), cv::Point(200, 499), cv::Point(399, 499),
      cv::Point(399, 579), cv::Point(579, 579), cv::Point(579, 399),
      cv::Point(499, 399), cv::Point(499, 200), cv::Point(579, 200),
      cv::Point(579, 20),  cv::Point(349, 20),  cv::Point(349, 100),
      cv::Point(250, 100), cv::Point(250, 20)};
  std::vector<cv::Point> handcrafted_polygon_4_2 = {
      cv::Point(220, 220), cv::Point(220, 380), cv::Point(380, 380),
      cv::Point(380, 220)};
  std::vector<std::vector<cv::Point>> contours = {handcrafted_polygon_4_1,
                                                  handcrafted_polygon_4_2};
  return contours;
}

std::vector<std::vector<cv::Point>> ConstructHandcraftedContours5() {
  std::vector<cv::Point> handcrafted_polygon_5_1 = {
      cv::Point(125, 50), cv::Point(50, 125), cv::Point(125, 200),
      cv::Point(200, 125)};
  std::vector<cv::Point> handcrafted_polygon_5_2 = {
      cv::Point(80, 300), cv::Point(80, 400), cv::Point(160, 400),
      cv::Point(120, 350), cv::Point(160, 300)};
  std::vector<cv::Point> handcrafted_polygon_5_3 = {
      cv::Point(100, 450), cv::Point(100, 550), cv::Point(140, 550),
      cv::Point(140, 450)};
  std::vector<cv::Point> handcrafted_polygon_5_4 = {
      cv::Point(300, 150), cv::Point(300, 250), cv::Point(400, 220),
      cv::Point(400, 180)};
  std::vector<std::vector<cv::Point>> contours = {
      handcrafted_polygon_5_1, handcrafted_polygon_5_2, handcrafted_polygon_5_3,
      handcrafted_polygon_5_4};
  return contours;
}
/*---------------------End 地图数据构造----------------------*/


/*---------------------Start Test Example-----------------------------------*/




//计算机器人旋转半径所占栅格数
int ComputeRobotRadius(const double& meters_per_pix,
                       const double& robot_size_in_meters) {
  int robot_radius = int(robot_size_in_meters / meters_per_pix);
  return robot_radius;
}
