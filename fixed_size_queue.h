#ifndef FIXED_SIZE_QUEUE_H
#define FIXED_SIZE_QUEUE_H

#include <cassert>
#include <memory>

template<typename T, bool OwnMemory = true>
class fixed_size_queue {
 public:
  // Use one larger capacity to work well iterators.
  fixed_size_queue(int capacity)
      : capacity_(capacity + 1), buffer_(new T[capacity + 1]()) {
    static_assert(OwnMemory, "Must provide external storage or set OwnMemory");
  }

  fixed_size_queue(T* buffer, int capacity)
      : capacity_(capacity), buffer_(buffer) {
    static_assert(!OwnMemory, "Cannot provide buffer with OwnMemory");
  }

  ~fixed_size_queue() {
    if (OwnMemory) {
      delete[] buffer_;
    }
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

  T back() const {
    int index = (start_ + size_ - 1) % capacity_;
    assert(index >= 0);
    assert(index < capacity_);
    return buffer_[index];
  }

  T front() const {
    assert(start_ >= 0);
    assert(start_ < capacity_);
    return buffer_[start_];
  }

  int size() const { return size_; }

  void shrink_to_size(int size) {
    assert(size_ >= size);
    size_ = size;
  }

  int capacity() const { return capacity_ - 1; }

  T get(int index) const {
    assert(index >= 0);
    assert(index < capacity_);
    return buffer_[(start_ + index) % capacity_];
  }

  T operator[](int index) const {
    return get(index);
  }

  class iterator {
   public:
    typedef T value_type;
    typedef T& reference;

    iterator(const fixed_size_queue<T, OwnMemory>& container, int index)
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
    const fixed_size_queue<T, OwnMemory>& container_;
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
