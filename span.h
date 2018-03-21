#ifndef SPAN_H
#define SPAN_H

#include <vector>

template<typename IndexType = unsigned int>
class ArraySpan {
 public:
  ArraySpan(IndexType start, IndexType size) : start_(start), size_(size) {}

  IndexType size() const {
    return size_;
  }

  IndexType start() const {
    return start_;
  }

 private:
  const IndexType size_;
  const IndexType start_;
};

#endif  // SPAN_H