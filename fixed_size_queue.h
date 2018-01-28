#ifndef FIXED_SIZE_QUEUE_H
#define FIXED_SIZE_QUEUE_H

#include <memory>

template<typename T>
class fixed_size_queue {
 public:
  // Use one larger capacity to work well iterators.
  fixed_size_queue(int capacity)
      : capacity_(capacity + 1), buffer_(new T[capacity]) {}

  void push(T element) {
    buffer_[(start_ + size_++) % capacity_] = element;
  }

  T pop() {
    T result = buffer_[start_];
    --size_;
    start_ = (start_ + 1) % capacity_;
    return result;
  }

  T back() {
    return buffer_[(start_ + size_ - 1) % capacity_];
  }

  T front() {
    return buffer_[start_];
  }

  int size() {
    return size_;
  }

  class iterator {
   public:
    typedef T value_type;
    typedef T& reference;

    iterator(const fixed_size_queue<T>& container, int index)
        : container_(container), index_(index) {}

    iterator(const iterator& other)
        : container_(other.container_), index_(other.index_) {}

    iterator operator++() {
      iterator it = *this;
      index_ = (index_ + 1) % container_.capacity_;
      return it;
    }

    reference operator*() {
      return container_.buffer_.get()[index_];
    }

   private:
    const fixed_size_queue<T>& container_;
    int index_;
  };

  iterator begin() {
    return iterator(*this, start_);
  }

  iterator end() {
    return iterator(*this, (start_ + size_) % capacity_);
  }

 private:
  const int capacity_;
  int size_ = 0;
  int start_ = 0;

  std::unique_ptr<T[]> buffer_;
};

#endif  // FIXED_SIZE_QUEUE_H
