#ifndef FIXED_SIZE_QUEUE_H
#define FIXED_SIZE_QUEUE_H

#include <cassert>
#include <memory>

template<typename T>
class fixed_size_queue {
 public:
  // Use one larger capacity to work well iterators.
  fixed_size_queue(int capacity)
      : capacity_(capacity + 1), buffer_(new T[capacity + 1]()) {}

  ~fixed_size_queue() {
    delete[] buffer_;
  }

  void push(T element) {
    assert(size_ < capacity_ - 1);
    int index = (start_ + size_++) % capacity_;
    assert(index >= 0);
    assert(index < capacity_);
    buffer_[index] = element;
  }

  T pop() {
    assert(size_ > 0);
    assert(size_ < capacity_);
    assert(start_ >= 0);
    assert(start_ < capacity_);

    T result = buffer_[start_];
    --size_;
    start_ = (start_ + 1) % capacity_;
    return result;
  }

  T back() {
    int index = (start_ + size_ - 1) % capacity_;
    assert(index >= 0);
    assert(index < capacity_);
    return buffer_[index];
  }

  T front() {
    assert(start_ >= 0);
    assert(start_ < capacity_);
    return buffer_[start_];
  }

  int size() { return size_; }

  int capacity() { return capacity_ - 1; }

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
      ++index_;
      return it;
    }

    reference operator*() const {
      assert(index_ >= 0);
      assert(index_ < container_.size_);
      return container_.buffer_[(container_.start_ + index_) % container_.capacity_];
    }

    bool operator==(const iterator& other) const {
      return &container_ == &other.container_ && index_ == other.index_;
    }

    bool operator!=(const iterator& other) const {
      return !(operator==(other));
    }

   private:
    const fixed_size_queue<T>& container_;
    int index_;
  };

  iterator begin() const {
    return iterator(*this, 0);
  }

  iterator end() const {
    return iterator(*this, size_);
  }

 private:
  const int capacity_;
  int size_ = 0;
  int start_ = 0;
  T* buffer_;
};

#endif  // FIXED_SIZE_QUEUE_H
