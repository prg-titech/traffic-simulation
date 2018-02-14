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
  static const uint32_t kTurnLane = 1;

  enum Type {
    // Sorted from smallest to largest.
    kResidential,
    kTertiary,
    kSecondary,
    kPrimary,
    kMotorwayLink,
    kMotorway,

    kMaxType
  };

  Cell(int max_velocity, double x, double y, Type type = kResidential,
       uint32_t tag = 0)
      : is_free_(true), max_velocity_(max_velocity), type_(type),
        controller_max_velocity_(max_velocity), x_(x), y_(y), tag_(tag) {
    assert(type >= 0 && type < Type::kMaxType);
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

  // Return max. velocity regardless of traffic controllers.
  int street_max_velocity() {
    return max_velocity_;
  }

  void draw();

  void occupy();

  void release();

  void connect_to(Cell* other) {
    outgoing_cells_.push_back(other);
    other->incoming_cells_.push_back(this);
  }

  int num_outgoing_cells() {
    assert(outgoing_cells_.size() > 0);
    return outgoing_cells_.size();
  }

  Cell** outgoing_cells() {
    return outgoing_cells_.data();
  }

  int num_incoming_cells() {
    return incoming_cells_.size();
  }

  Cell** incoming_cells() {
    return incoming_cells_.data();
  }

  double x() { return x_; }
  double y() { return y_; }

  Type type() { return type_; }

  uint32_t tag() { return tag_; }

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
  Type type_;
  bool is_free_;
  int max_velocity_;
  int controller_max_velocity_;

  std::vector<Cell*> outgoing_cells_;
  std::vector<Cell*> incoming_cells_;

  // Coordinates on the map. Used for rendering.
  double x_, y_;

  // Debug information only.
  uint32_t tag_;
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


class TrafficController {
 public:
  virtual void step() = 0;
};


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


class PriorityYieldTrafficController : public TrafficController {
 public:
  PriorityYieldTrafficController(std::vector<Cell*> cells) : cells_(cells) {}

  void step();

 private:
  std::vector<Cell*> cells_;

  // Check if a car is coming from this cell within the next iteration.
  bool has_incoming_traffic(Cell* cell);
  bool has_incoming_traffic(Cell* cell, int lookahead);
};


class Street {
 public:
  Street(Cell* first, Cell* last, Cell::Type type = Cell::kResidential)
      : first_(first), last_(last), type_(type) {
    assert(type >= 0 && type < Cell::kMaxType);
  }

  Cell* first() { return first_; }
  Cell* last() { return last_; }
  Cell::Type type() { return type_; }

 private:
  Cell* first_;
  Cell* last_;
  Cell::Type type_;
};

class Simulation {
 public:
  std::vector<Cell*>& cells() { return cells_; }

  int num_cells() { return cells_.size(); }

  Cell* random_cell() { return cells_[rand() % num_cells()]; }

  std::vector<Street*>& streets() { return streets_; }

  void step() {
    for (int i = 0; i < traffic_controllers_.size(); ++i) {
      traffic_controllers_[i]->step();
    }

    for (int i = 0; i < cars_.size(); ++i) {
      cars_[i]->step_velocity();
    }

    for (int i = 0; i < cars_.size(); ++i) {
      cars_[i]->step_move();
    }
  }

  void add_street(Street* street) { streets_.push_back(street); }
  void add_cell(Cell* cell) { cells_.push_back(cell); }
  void add_car(Car* car) { cars_.push_back(car); }
  void add_traffic_controller(TrafficController* light) { 
    traffic_controllers_.push_back(light);
  }

  // Currently only for GUI purposes.
  std::vector<Street*> streets_;
  std::vector<Cell*> cells_;
  std::vector<Car*> cars_;
  std::vector<TrafficController*> traffic_controllers_;
};

#endif  // TRAFFIC_H
