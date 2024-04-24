#ifndef NAVIGATION_MESSAGE_
#define NAVIGATION_MESSAGE_
#include "math_utils.h"
using namespace math_utils;
class NavigationMessage {
 public:
  NavigationMessage() {
    forward_distance = 0.0;
    global_yaw_angle = 0.0;
    local_yaw_angle = 0.0;
  }
  void SetDistance(double dist) { forward_distance = dist; }
  void SetGlobalYaw(double global_yaw) { global_yaw_angle = global_yaw; }
  void SetLocalYaw(double local_yaw) { local_yaw_angle = local_yaw; }

  double GetDistance() { return forward_distance; }

  double GetGlobalYaw() { return global_yaw_angle; }

  double GetLocalYaw() { return local_yaw_angle; }

  void GetMotion(double& dist, double& global_yaw, double& local_yaw) {
    dist = forward_distance;
    global_yaw = global_yaw_angle;
    local_yaw = local_yaw_angle;
  }
  void Reset() {
    forward_distance = 0.0;
    global_yaw_angle = 0.0;
    local_yaw_angle = 0.0;
  }

 private:
  double forward_distance;
  // 欧拉角表示，逆时针为正，顺时针为负
  double global_yaw_angle;
  double local_yaw_angle;
};
/**
 * @brief 根据给定的当前方向、路径和地图分辨率，生成导航消息队列。
 * 每个导航消息包含了路径上一段连续直线的信息，包括该段直线的全局偏航角、局部偏航角和距离
 *根据全局偏航角的变化，将路径分割成一系列连续直线段，并为每个直线段生成一个导航消息。
 * @param curr_direction 当前方向
 * @param pos_path 路径
 * @param meters_per_pix 地图分辨率
 * @return std::vector<NavigationMessage>
 */
inline std::vector<NavigationMessage> GetNavigationMessage(
    const Eigen::Vector2d& curr_direction, std::deque<Point2D> pos_path,
    double meters_per_pix) {
  // global_base_direction: 全局基方向，初始化为向下的方向。
  Eigen::Vector2d global_base_direction = {0, -1};  // {x, y}
  // local_base_direction: 局部基方向，初始化为当前方向
  Eigen::Vector2d local_base_direction = curr_direction;
  // 当前局部方向和全局方向
  Eigen::Vector2d curr_local_direction;
  Eigen::Vector2d curr_global_direction;

  NavigationMessage message;
  std::vector<NavigationMessage> message_queue;

  double distance = 0.0;
  double step_distance = 0.0;
  // 计算当前方向与基方向之间的偏航角
  double prev_global_yaw = ComputeYaw(curr_direction, global_base_direction);
  double curr_global_yaw = 0.0;
  double curr_local_yaw = 0.0;

  message.SetGlobalYaw(DBL_MAX);
  message.SetLocalYaw(DBL_MAX);

  for (int i = 0; i < pos_path.size() - 1; i++) {
    if (pos_path[i + 1] == pos_path[i]) {
      continue;
    } else {
      // 计算当前局部方向向量
      curr_local_direction = {pos_path[i + 1].x - pos_path[i].x,
                              pos_path[i + 1].y - pos_path[i].y};
      curr_local_direction.normalize();  //向量归一化
      // 计算当前局部方向与基方向之间的偏航角
      curr_global_yaw = ComputeYaw(curr_local_direction, global_base_direction);
      // 计算当前局部方向与上一步局部方向之间的偏航角
      curr_local_yaw = ComputeYaw(curr_local_direction, local_base_direction);

      if (message.GetGlobalYaw() == DBL_MAX)  // initialization
      {
        message.SetGlobalYaw(curr_global_yaw);
      }

      if (message.GetLocalYaw() == DBL_MAX)  // initialization
      {
        message.SetLocalYaw(curr_local_yaw);
      }
      // 如果当前全局偏航角与上一步的全局偏航角相同
      if (curr_global_yaw == prev_global_yaw) {
        // 计算当前步长，并累加
        step_distance =
            ComputeDistance(pos_path[i + 1], pos_path[i], meters_per_pix);
        distance += step_distance;
      } else {
        // 如果当前全局偏航角与上一步的全局偏航角不同
        // 将上一步积累的距离添加到导航消息队列中
        message.SetDistance(distance);
        message_queue.emplace_back(message);

        //重置，设置新的全局偏航角和局部偏航角
        message.Reset();
        message.SetGlobalYaw(curr_global_yaw);
        message.SetLocalYaw(curr_local_yaw);

        distance = 0.0;
        step_distance =
            ComputeDistance(pos_path[i + 1], pos_path[i], meters_per_pix);
        distance += step_distance;
      }
      // 更新上一步全局偏航角及局部基方向
      prev_global_yaw = curr_global_yaw;

      local_base_direction = curr_local_direction;
    }
  }
  // 将最后一步积累的距离添加到导航消息队列中
  message.SetDistance(distance);
  message_queue.emplace_back(message);

  return message_queue;
}

inline void PrintMotionCommands(
    const std::vector<NavigationMessage>& navigation_messages) {
  double dist = 0.0, global_yaw = 0.0, local_yaw = 0.0;
  for (auto message : navigation_messages) {
    message.GetMotion(dist, global_yaw, local_yaw);
    std::cout << "globally rotate " << global_yaw << " degree(locally rotate "
              << local_yaw << " degree) and go forward for " << dist << " m."
              << std::endl;
  }
}

#endif  // !NAVIGATION_MESSAGE_
