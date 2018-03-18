#ifndef SPAN_H
#define SPAN_H

#include <vector>

template<typename T, typename IndexType = unsigned int>
class Span {
 public:
  Span(T* data, IndexType size) : size_(size), data_(data) {}
  Span(const std::vector<T>& vec) : size_(vec.size()), data_(vec.data()) {}

  IndexType size() {
    return size_;
  }

  T operator[](IndexType index) {
    return data_[index];
  }

 private:
  const IndexType size_;
  const T* data_;
};

#endif  // SPAN_H