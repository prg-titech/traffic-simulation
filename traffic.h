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
  Cell(double max_velocity, double x, double y)
      : is_free_(true), max_velocity_(max_velocity),
        controller_max_velocity_(max_velocity), x_(x), y_(y) {}

  bool is_free() const {
    return is_free_;
  }

  double max_velocity() const {
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

  double x() { return x_; }
  double y() { return y_; }

  void set_max_velocity(double velocity) {
    max_velocity_ = controller_max_velocity_ = velocity;
  }

  void set_controller_max_velocity(double velocity) {
    controller_max_velocity_ = velocity;
  }

  void remove_controller_max_velocity() {
    controller_max_velocity_ = max_velocity_;
  }

 private:
  bool is_free_;
  double max_velocity_;
  double controller_max_velocity_;

  std::vector<Cell*> neighbors_;

  // Coordinates on the map. Used for rendering.
  double x_, y_;
};


class Car {
 public:
  Car(double max_velocity, Cell* initial_position)
      : max_velocity_(max_velocity), path_(max_velocity),
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
  double velocity_ = 0;
  double max_velocity_;

  // Maintain max_velocity_ many cells in the queue. This is the path that the
  // car is going to take. The maximum movement speed is limited by the
  // maximum velocity of every path cell.
  fixed_size_queue<Cell*> path_;

  Cell* position_;
};


class SharedSignalGroup {
 public:
  // TODO: Use rvalue references.
  SharedSignalGroup(std::vector<Cell*> cells) : cells_(cells) {}

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
               std::vector<SharedSignalGroup*> signal_groups)
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

class Simulation {
 public:
  using StreetLine = std::tuple<double, double, double, double>;

  std::vector<Cell*>& cells() { return cells_; }

  int num_cells() { return cells_.size(); }

  Cell* random_cell() { return cells_[rand() % num_cells()]; }

  std::vector<StreetLine>& streets() { return streets_; }

  void step() {
    for (int i = 0; i < traffic_lights_.size(); ++i) {
      traffic_lights_[i]->step();
    }

    for (int i = 0; i < cars_.size(); ++i) {
      cars_[i]->step_velocity();
    }

    for (int i = 0; i < cars_.size(); ++i) {
      cars_[i]->step_move();
    }
  }

  void add_street(StreetLine street) { streets_.push_back(street); }
  void add_cell(Cell* cell) { cells_.push_back(cell); }
  void add_car(Car* car) { cars_.push_back(car); }
  void add_traffic_light(TrafficLight* light) { 
    traffic_lights_.push_back(light);
  }

  // Only for GUI purposes.
  std::vector<StreetLine> streets_;

  std::vector<Cell*> cells_;
  std::vector<Car*> cars_;
  std::vector<TrafficLight*> traffic_lights_;
};

#endif  // TRAFFIC_H
