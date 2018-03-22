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

// TODO: Check if the compiler will optimize out Span.
// TODO: The const of data might be a bit too restrictive here...
template<typename T, typename IndexType = unsigned int>
class Span {
 public:
  Span(const T* data, IndexType size) : data_(data), size_(size) {}

  T* data() const { return data_; }

  IndexType size() const { return size_; }

  const T& operator[](IndexType index) const {
    return data_[index];
  }

 private:
  const T* const data_;
  const IndexType size_;
};

#endif  // SPAN_H
