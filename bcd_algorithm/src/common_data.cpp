#include "common_data.h"

namespace CommonData {

//运算符重载，比较Point2D类型的点大小

bool Point2D::operator<(const Point2D& other) const {
  return (x < other.x) || (x == other.x && y < other.y);
}

bool Point2D::operator==(const Point2D& other) const {
  return x == other.x && y == other.y;
}

bool Point2D::operator!=(const Point2D& other) const {
  return x != other.x || y != other.y;
  //使用了成员访问运算符 *this 来表示当前对象，并与 other 对象进行比较
  //   return !(*this == other);
}

//运算符重载，比较Event大小
bool Event::operator<(const Event& other) const {
  return x < other.x || (x == other.x && y < other.y) ||
         (x == other.x && y == other.y &&
          obstacle_index < other.obstacle_index);
}

}  // namespace CommonData
