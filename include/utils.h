#ifndef UTILS_H
#define UTILS_H

#include <vector>

namespace utils {
template <class T>
T avg(const std::vector<T> datas) {
  T sum_data = T(0);
  for (auto data : datas) sum_data += data;
  return sum_data / datas.size();
}
}  // namespace utils

#endif  // UTILS_H
