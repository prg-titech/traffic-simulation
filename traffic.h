#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <cassert>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <vector>
#include <set>

#include "fixed_size_queue.h"
#include "span.h"

class Renderer;

namespace simulation {
namespace standard {

using IndexType = unsigned int;

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

  Cell(IndexType id, int max_velocity, double x, double y,
       Type type = kResidential, uint32_t tag = 0);

  // Returns true if the cell is free.
  bool is_free() const;

  // Returns the maximum velocity allowed on this cell at this moment. This
  // function takes into account velocity limitations due to traffic lights.
  int max_velocity() const;

  // Return max. velocity regardless of traffic controllers.
  int street_max_velocity() const;

  // Draw this cell on the GUI.
  void draw() const;

  // A car enters this cell.
  void occupy(Car* car);

  // A car leaves this cell.
  void release();

  // Connects this cell to another cell.
  void connect_to(Cell* other);

  const Span<Cell*> outgoing_cells() const {
    return Span<Cell*>(outgoing_cells_.data(), outgoing_cells_.size());
  }

  const Span<Cell*> incoming_cells() const {
    return Span<Cell*>(incoming_cells_.data(), incoming_cells_.size());
  }

  // Return x and y coordinates of this cell. For rendering purposes only.
  double x() const { return x_; }
  double y() const { return y_; }

  // Returns the type of this cell.
  Type type() const { return type_; }

  // Additional information can be stored in the tag.
  uint32_t tag() const { return tag_; }

  // Sets the maximum velocity allowed on this street. This function will
  // override any speed limits imposed by a traffic controller.
  void set_max_velocity(int velocity) {
    max_velocity_ = controller_max_velocity_ = velocity;
  }

  // Sets the maximum temporary speed limit (traffic controller).
  void set_controller_max_velocity(int velocity) {
    controller_max_velocity_ = velocity;
  }

  // Removes the maximum temporary speed limit.
  void remove_controller_max_velocity() {
    controller_max_velocity_ = max_velocity_;
  }

  // Returns the car that occupies this cell.
  Car* car() const { return car_; }

  // Make this cell a sink.
  void set_sink(bool is_sink) { is_sink_ = is_sink; }

  // Returns true if this cell is a sink.
  bool is_sink() const { return is_sink_; }

  IndexType id() const { return id_; }

 private:
  const IndexType id_;

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
  Car(IndexType id, int max_velocity, Cell* initial_position,
      uint32_t random_state)
      : id_(id), max_velocity_(max_velocity), path_(max_velocity),
        position_(initial_position), is_active_(true),
        random_state_(random_state) {
    initial_position->occupy(this);
  }

  void assert_check_velocity() const;

  void step_velocity();

  void step_move();

  Cell* position() const { return position_; }

  void set_position(Cell* cell);

  bool is_active() const { return is_active_; }

  bool is_jammed() const;

  void set_active(bool is_active) { is_active_ = is_active; }

  int velocity() const { return velocity_; }

  // Path cannot be modified using this API. It can only be modified from
  // within this class.
  const fixed_size_queue<Cell*>& path() const { return path_; }

  IndexType id() const { return id_; }

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
  const IndexType id_;

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
  uint32_t& random_state() { return random_state_; }
};


class SharedSignalGroup {
 public:
  // TODO: Use rvalue references.
  SharedSignalGroup(IndexType id, std::vector<Cell*> cells)
      : id_(id), cells_(cells) {}

  // Sets traffic lights to green.
  void signal_go();

  // Sets traffic lights to red.
  void signal_stop();

  // Returns a vector of cells that belong to this group.
  const std::vector<Cell*>& cells() { return cells_; }

  IndexType id() const { return id_; }

 private:
  const IndexType id_;

  const std::vector<Cell*> cells_;
};


class TrafficController {
 public:
  virtual void initialize() = 0;
  virtual void step() = 0;
  virtual void assert_check_state() const = 0;
};


class TrafficLight : public TrafficController {
 public:
  TrafficLight(IndexType id, int phase_time,
               std::vector<SharedSignalGroup*> signal_groups);

  // Set all lights to red.
  void initialize();

  void step();

  // Make sure that only one group has green light.
  void assert_check_state() const;

  IndexType id() { return id_; }

 private:
  const IndexType id_;

  // This timer is increased with every step.
  int timer_;

  // The number of cycles in timer_ until the signal group is changed.
  const int phase_time_;

  // Index into groups_. The specified signal group has a green light.
  int phase_ = 0;

  // Cells which are set to "green" at the same time.
  const std::vector<SharedSignalGroup*> signal_groups_;
};


class PriorityYieldTrafficController : public TrafficController {
 public:
  PriorityYieldTrafficController(
      IndexType id, std::vector<SharedSignalGroup*> groups)
      : id_(id), groups_(groups) {}

  void initialize() {}

  void step();

  void assert_check_state() const;

  IndexType id() const { return id_; }

 private:
  const IndexType id_;

  const std::vector<SharedSignalGroup*> groups_;

  // Check if a car is coming from this group within the next iteration.
  bool has_incoming_traffic(SharedSignalGroup* group) const;
  bool has_incoming_traffic(Cell* cell, int lookahead) const;
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

  // Initialize this traffic simulation. May be called only when all streets
  // cars, traffic controllers, etc. were added.
  void initialize();

  Cell* random_cell(uint32_t* state) const;

  Cell* random_free_cell(uint32_t* state) const;

  // Simulate a single tick.
  void step();

  void add_street(Street* street) {
    streets_.push_back(street);
  }

  void add_cell(Cell* cell) {
    cells_.push_back(cell);
  }
  
  void add_car(Car* car) {
    cars_.push_back(car);
  }

  void add_traffic_controller(TrafficController* light) {
    traffic_controllers_.push_back(light);
  }

  void add_inactive_car(Car* car) {
    inactive_cars_.push_back(car);
  }

  // Return a vector of all cars. Only used for debug output.
  const std::vector<Car*>& cars() const { return cars_; }

  // Print information about this simulation.
  void print_stats() const;

  // Calculate a checksum for the state of this simulation.
  uint64_t checksum() const;

 private:
  friend class ::Renderer;

  void step_cells();
  void step_traffic_controllers();
  void step_cars();

  // A vector of all streets. Contains only the cells of start and
  // end points. Only used for GUI purposes.
  std::vector<Street*> streets_;
  const std::vector<Street*>& streets() const {
    return streets_;
  }

  // A vector of all cells.
  std::vector<Cell*> cells_;
  const std::vector<Cell*>& cells() const { return cells_; }

  // A vector of all cars.
  std::vector<Car*> cars_;

  // A vector of all traffic controllers, e.g., traffic lights.
  std::vector<TrafficController*> traffic_controllers_;

  // A vector of all inactive cars. Used to keep track of cars that are
  // leaving the map.
  std::vector<Car*> inactive_cars_;
};

}  // namespace standard
}  // namespace simulation

#endif  // TRAFFIC_H
