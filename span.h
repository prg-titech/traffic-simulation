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

  IndexType operator[](IndexType index) const {
    return start_ + index;
  }

  class iterator {
   public:
    typedef IndexType value_type;
    typedef IndexType reference;

    iterator(IndexType state) : state_(state) {}

    iterator operator++() {
      iterator it = *this;
      ++state_;
      return it;
    }

    reference operator*() const {
      return state_;
    }

    bool operator==(const iterator& other) const {
      return state_ == other.state_;
    }

    bool operator!=(const iterator& other) const {
      return !(operator==(other));
    }

   private:
    IndexType state_;
  };

  iterator begin() const {
    return iterator(start_);
  }

  iterator end() const {
    return iterator(start_ + size_);
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

  const T* data() const { return data_; }

  IndexType size() const { return size_; }

  const T& operator[](IndexType index) const {
    return data_[index];
  }

 private:
  const T* const data_;
  const IndexType size_;
};

#endif  // SPAN_H
