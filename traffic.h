#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <cassert>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <vector>

#include "fixed_size_queue.h"


// Traffic simulation based on cellular automaton
class Cell {
 public:
  Cell(int max_velocity, int x, int y)
      : is_free_(true), max_velocity_(max_velocity),
        controller_max_velocity_(max_velocity), x_(x), y_(y) {
    assert(x >= 0);
    assert(y >= 0);
  }

  bool is_free() const {
    return is_free_;
  }

  int max_velocity() const {
    if (controller_max_velocity_ < max_velocity_) {
      return controller_max_velocity_;
    } else {
      return max_velocity_;
    }
  }

  void draw();

  void occupy();

  void release();

  void connect_to(Cell* other) {
    neighbors_.push_back(other);
  }

  int num_neighbors() {
    assert(neighbors_.size() > 0);
    return neighbors_.size();
  }

  Cell** neighbors() {
    return neighbors_.data();
  }

  int x() { return x_; }
  int y() { return y_; }

  void set_max_velocity(int velocity) {
    max_velocity_ = controller_max_velocity_ = velocity;
  }

  void set_controller_max_velocity(int velocity) {
    controller_max_velocity_ = velocity;
  }

  void remove_controller_max_velocity() {
    controller_max_velocity_ = max_velocity_;
  }

 private:
  bool is_free_;
  int max_velocity_;
  int controller_max_velocity_;

  std::vector<Cell*> neighbors_;

  // Coordinates on the map. Used for rendering.
  int x_, y_;
};


class Car {
 public:
  Car(int max_velocity, Cell* initial_position) : max_velocity_(max_velocity),
                                                  path_(max_velocity),
                                                  position_(initial_position) {
    initial_position->occupy();
  }

  void step_velocity();

  void step_move();

  Cell* position() { return position_; }

 protected:
  // Assuming that the car is located at position, determine where to go next.
  Cell* next_step(Cell* position);

  void step_accelerate();

  void step_extend_path();

  void step_constraint_velocity();

  void step_slow_down();

 private:
  int velocity_ = 0;
  int max_velocity_;

  // Maintain max_velocity_ many cells in the queue. This is the path that the
  // car is going to take. The maximum movement speed is limited by the
  // maximum velocity of every path cell.
  fixed_size_queue<Cell*> path_;

  Cell* position_;
};


class SharedSignalGroup {
 public:
  SharedSignalGroup(std::initializer_list<Cell*> cells) : cells_(cells) {}

  // Set traffic lights to green.
  void signal_go();

  // Set traffic lights to red.
  void signal_stop();

 private:
  std::vector<Cell*> cells_;
};


class TrafficController {};

class TrafficLight : public TrafficController {
 public:
  TrafficLight(int phase_time,
               std::initializer_list<SharedSignalGroup*> signal_groups)
      : phase_time_(phase_time), signal_groups_(signal_groups) {}

  void step();

  // Set all lights to red.
  void initialize();

 private:
  int timer_;
  const int phase_time_;

  // Index into groups_. The specified signal group has a green light.
  int phase_ = 0;

  // Cells which are set to "green" at the same time.
  std::vector<SharedSignalGroup*> signal_groups_;
};

#endif  // TRAFFIC_H
