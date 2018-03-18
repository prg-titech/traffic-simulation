#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <cassert>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <vector>
#include <set>

#include "fixed_size_queue.h"


class Car;

// Cell for traffic flow simulation based on cellular automaton.
class Cell {
 public:
  static const uint32_t kTurnLane = 1;

  // Cell type according to OSM data.
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
       uint32_t tag = 0);

  // Returns true if the cell is free.
  bool is_free() const;

  // Returns the maximum velocity allowed on this cell at this moment. This
  // function takes into account velocity limitations due to traffic lights.
  int max_velocity() const;

  // Return max. velocity regardless of traffic controllers.
  int street_max_velocity() const {
    return max_velocity_;
  }

  // Draw this cell on the GUI.
  void draw() const;

  void occupy(Car* car);

  void release();

  void connect_to(Cell* other);

  const std::vector<Cell*>& outgoing_cells() const {
    return outgoing_cells_;
  }

  const std::vector<Cell*>& incoming_cells() const {
    return incoming_cells_;
  }

  double x() const { return x_; }
  double y() const { return y_; }

  Type type() const { return type_; }

  uint32_t tag() const { return tag_; }

  void set_max_velocity(int velocity) {
    max_velocity_ = controller_max_velocity_ = velocity;
  }

  void set_controller_max_velocity(int velocity) {
    controller_max_velocity_ = velocity;
  }

  void remove_controller_max_velocity() {
    controller_max_velocity_ = max_velocity_;
  }

  Car* car() const { return car_; }

  void set_sink(bool is_sink) { is_sink_ = is_sink; }

  bool is_sink() const { return is_sink_; }

 private:
  Type type_;
  bool is_free_;
  bool is_sink_ = false;
  int max_velocity_;
  int controller_max_velocity_;

  std::vector<Cell*> outgoing_cells_;
  std::vector<Cell*> incoming_cells_;

  Car* car_ = nullptr;

  // Coordinates on the map. Used for rendering.
  double x_, y_;

  // Debug information only.
  uint32_t tag_;
};


class Car {
 public:
  Car(int max_velocity, Cell* initial_position, uint32_t random_state)
      : max_velocity_(max_velocity), path_(max_velocity),
        position_(initial_position), is_active_(true),
        random_state_(random_state) {
    initial_position->occupy(this);
  }

  void assert_check_velocity() const;

  void step_velocity();

  void step_move();

  Cell* position() const { return position_; }

  bool is_jammed() const;

  void set_active(bool is_active) { is_active_ = is_active; }

  bool is_active() const { return is_active_; }

  int velocity() const { return velocity_; }

  // Path cannot be modified using this API. It can only be modified from
  // within this class.
  const fixed_size_queue<Cell*>& path() const { return path_; }

  void set_position(Cell* cell);

 protected:
  friend class Simulation;

  // Assuming that the car is located at position, determine where to go next.
  Cell* next_step(Cell* position);

  void step_initialize_iteration();

  void step_accelerate();

  void step_extend_path();

  void step_constraint_velocity();

  void step_slow_down();

 private:
  bool is_active_;
  int velocity_ = 0;
  int max_velocity_;

  // Maintain max_velocity_ many cells in the queue. This is the path that the
  // car is going to take. The maximum movement speed is limited by the
  // maximum velocity of every path cell.
  fixed_size_queue<Cell*> path_;

  Cell* position_;

  // Every car has a state for its random number generator.
  uint32_t random_state_;
  uint32_t rand32();
};


class SharedSignalGroup {
 public:
  // TODO: Use rvalue references.
  SharedSignalGroup(std::vector<Cell*> cells) : cells_(cells) {}

  // Set traffic lights to green.
  void signal_go();

  // Set traffic lights to red.
  void signal_stop();

  const std::vector<Cell*>& cells() const { return cells_; }

 private:
  std::vector<Cell*> cells_;
};


class TrafficController {
 public:
  virtual void initialize() = 0;
  virtual void step() = 0;
  virtual void assert_check_state() const = 0;
};


class TrafficLight : public TrafficController {
 public:
  TrafficLight(int phase_time,
               std::vector<SharedSignalGroup*> signal_groups);
  void step();

  // Set all lights to red.
  void initialize();

  // Make sure that only one group has green light.
  void assert_check_state() const;

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
  PriorityYieldTrafficController(std::vector<SharedSignalGroup*> groups)
      : groups_(groups) {}

  void initialize() {}

  void step();

  void assert_check_state() const;

 private:
  std::vector<SharedSignalGroup*> groups_;

  // Check if a car is coming from this group within the next iteration.
  bool has_incoming_traffic(SharedSignalGroup* group);
  bool has_incoming_traffic(Cell* cell, int lookahead);
};


class Street {
 public:
  Street(Cell* first, Cell* last, Cell::Type type = Cell::kResidential)
      : first_(first), last_(last), type_(type) {
    assert(type >= 0 && type < Cell::kMaxType);
  }

  Cell* first() const { return first_; }
  Cell* last() const { return last_; }
  Cell::Type type() const { return type_; }

 private:
  Cell* first_;
  Cell* last_;
  Cell::Type type_;
};

class Simulation {
 public:
  Simulation() {}

  void initialize();

  Cell* random_cell(uint32_t* state) const;

  Cell* random_free_cell(uint32_t* state) const;

  void step();

  void add_street(Street* street) { streets_.push_back(street); }
  void add_cell(Cell* cell) { cells_.push_back(cell); }
  void add_car(Car* car) { cars_.push_back(car); }
  void add_traffic_controller(TrafficController* light) { 
    traffic_controllers_.push_back(light);
  }
  void add_inactive_car(Car* car) { inactive_cars_.push_back(car); }

  // Return a vector of all cars. Only used for debug output.
  const std::vector<Car*>& cars() const { return cars_; }

  // Print information about this simulation.
  void print_stats() const;

  // Calculate a checksum for the state of this simulation.
  uint64_t checksum() const;

 private:
  friend class Renderer;

  // A vector of all streets. Contains only the cells of start and
  // end points. Only used for GUI purposes.
  std::vector<Street*> streets_;
  std::vector<Street*>& streets() { return streets_; }

  // A vector of all cells.
  std::vector<Cell*> cells_;
  std::vector<Cell*>& cells() { return cells_; }

  // A vector of all cars.
  std::vector<Car*> cars_;

  // A vector of all traffic controllers, e.g., traffic lights.
  std::vector<TrafficController*> traffic_controllers_;

  // A vector of all inactive cars. Used to keep track of cars that are
  // leaving the map.
  std::vector<Car*> inactive_cars_;
};

#endif  // TRAFFIC_H
