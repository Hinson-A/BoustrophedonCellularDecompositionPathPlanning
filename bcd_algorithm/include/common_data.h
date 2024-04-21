#pragma once

#include <climits>
#include <deque>
#include <vector>
namespace CommonData {

//事件类型
enum EventType {
  //障碍物事件类型
  IN,
  IN_TOP,
  IN_BOTTOM,
  OUT,
  OUT_TOP,
  OUT_BOTTOM,
  INNER_IN,
  INNER_IN_TOP,
  INNER_IN_BOTTOM,
  INNER_OUT,
  INNER_OUT_TOP,
  INNER_OUT_BOTTOM,

  //墙事件类型
  IN_EX,
  IN_TOP_EX,
  IN_BOTTOM_EX,
  OUT_EX,
  OUT_TOP_EX,
  OUT_BOTTOM_EX,
  INNER_IN_EX,
  INNER_IN_TOP_EX,
  INNER_IN_BOTTOM_EX,
  INNER_OUT_EX,
  INNER_OUT_TOP_EX,
  INNER_OUT_BOTTOM_EX,

  MIDDLE,
  CEILING,
  FLOOR,
  UNALLOCATED
};

// 二维点结构体定义
struct Point2D {
  // 默认构造函数
  Point2D() : x(INT_MAX), y(INT_MAX) {}

  // 带参数的构造函数
  Point2D(int x, int y) : x(x), y(y) {}

  // 数据成员
  int x;
  int y;

  //!
  /*将运算符重载移动到源文件中并避免多重定义的问题，需要执行以下步骤：
   * 在头文件中仅保留运算符重载的声明。
   * 在源文件中实现这些运算符重载。
   */
  bool operator<(const Point2D& other) const;
  bool operator==(const Point2D& other) const;
  bool operator!=(const Point2D& other) const;
};

// 定义事件
struct Event {
  Event(int obstacle_idx, int x_pos, int y_pos, EventType type = UNALLOCATED)
      : obstacle_index(obstacle_idx),
        x(x_pos),
        y(y_pos),
        event_type(type),
        original_index_in_slice(INT_MAX),
        isUsed(false) {}
  int x;                        // 事件点的x坐标
  int y;                        // 事件点的y坐标
  int original_index_in_slice;  // 事件在切片中的原始索引
  int obstacle_index;           // 障碍物索引
  EventType event_type;         // 事件类型
  bool isUsed;                  // 事件是否被使用
  bool operator<(const Event& other) const;
};

typedef std::deque<Point2D> Edge;
// CellNode 结构体用于表示路径规划中的单元格节点。
struct CellNode {
  // 构造函数初始化所有成员变量
  CellNode()
      : isVisited(false),
        isCleaned(false),
        parentIndex(INT_MAX),
        cellIndex(INT_MAX) {}

  bool isVisited;  // 标记单元格是否已被访问过
  bool isCleaned;  // 标记单元格是否已被清理
  Edge ceiling;  // 单元格的上边界（可能是一个包含点的Edge对象或类似的数据结构）
  Edge floor;    // 单元格的下边界
  int parentIndex;  // 父节点的索引，在路径规划中用于回溯
  // 邻居节点的索引列表，表示与当前单元格相邻的单元格
  std::deque<int> neighbor_indices;
  int cellIndex;  // 当前单元格的索引
};

}  // namespace CommonData
