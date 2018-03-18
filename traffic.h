#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <cassert>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <vector>
#include <set>

#define OPTION_CLASS 1
#include "function_helper.h"
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
  FUNCTION_DECL(is_free, bool) const;

  // Returns the maximum velocity allowed on this cell at this moment. This
  // function takes into account velocity limitations due to traffic lights.
  FUNCTION_DECL(max_velocity, int) const;

  // Return max. velocity regardless of traffic controllers.
  FUNCTION_DECL(street_max_velocity, int) const;

  // Draw this cell on the GUI.
  FUNCTION_DECL(draw, void) const;

  // A car enters this cell.
  FUNCTION_DECL(occupy, void, Car* car);

  // A car leaves this cell.
  FUNCTION_DECL(release, void);

  // Connects this cell to another cell.
  FUNCTION_DECL(connect_to, void, Cell* other);

  FUNCTION_DECL(outgoing_cells, const std::vector<Cell*>&) const {
    return outgoing_cells_;
  }

  FUNCTION_DECL(incoming_cells, const std::vector<Cell*>&) const {
    return incoming_cells_;
  }

  // Return x and y coordinates of this cell. For rendering purposes only.
  FUNCTION_DECL(x, double) const { return x_; }
  FUNCTION_DECL(y, double) const { return y_; }

  // Returns the type of this cell.
  FUNCTION_DECL(type, Type) const { return type_; }

  // Additional information can be stored in the tag.
  FUNCTION_DECL(tag, uint32_t) const { return tag_; }

  // Sets the maximum velocity allowed on this street. This function will
  // override any speed limits imposed by a traffic controller.
  FUNCTION_DECL(set_max_velocity, void, int velocity) {
    max_velocity_ = controller_max_velocity_ = velocity;
  }

  // Sets the maximum temporary speed limit (traffic controller).
  FUNCTION_DECL(set_controller_max_velocity, void, int velocity) {
    controller_max_velocity_ = velocity;
  }

  // Removes the maximum temporary speed limit.
  FUNCTION_DECL(remove_controller_max_velocity, void) {
    controller_max_velocity_ = max_velocity_;
  }

  // Returns the car that occupies this cell.
  FUNCTION_DECL(car, Car*) const { return car_; }

  // Make this cell a sink.
  FUNCTION_DECL(set_sink, void, bool is_sink) { is_sink_ = is_sink; }

  // Returns true if this cell is a sink.
  FUNCTION_DECL(is_sink, bool) const { return is_sink_; }

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

  FUNCTION_DECL(assert_check_velocity, void) const;

  FUNCTION_DECL(step_velocity, void);

  FUNCTION_DECL(step_move, void);

  FUNCTION_DECL(position, Cell*) const { return position_; }

  FUNCTION_DECL(set_position, void, Cell* cell);

  FUNCTION_DECL(is_active, bool) const { return is_active_; }

  FUNCTION_DECL(is_jammed, bool) const;

  FUNCTION_DECL(set_active, void, bool is_active) { is_active_ = is_active; }

  FUNCTION_DECL(velocity, int) const { return velocity_; }

  // Path cannot be modified using this API. It can only be modified from
  // within this class.
  FUNCTION_DECL(path, const fixed_size_queue<Cell*>&) const { return path_; }

 protected:
  friend class Simulation;

  // Assuming that the car is located at position, determine where to go next.
  FUNCTION_DECL(next_step, Cell*, Cell* position);

  FUNCTION_DECL(step_initialize_iteration, void);

  FUNCTION_DECL(step_accelerate, void);

  FUNCTION_DECL(step_extend_path, void);

  FUNCTION_DECL(step_constraint_velocity, void);

  FUNCTION_DECL(step_slow_down, void);

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
  FUNCTION_DECL(rand32, uint32_t);
};


class SharedSignalGroup {
 public:
  // TODO: Use rvalue references.
  SharedSignalGroup(std::vector<Cell*> cells) : cells_(cells) {}

  // Sets traffic lights to green.
  FUNCTION_DECL(signal_go, void);

  // Sets traffic lights to red.
  FUNCTION_DECL(signal_stop, void);

  // Returns a vector of cells that belong to this group.
  FUNCTION_DECL(cells, const std::vector<Cell*>&) { return cells_; }

 private:
  const std::vector<Cell*> cells_;
};


class TrafficController {
 public:
  virtual FUNCTION_DECL(initialize, void) = 0;
  virtual FUNCTION_DECL(step, void) = 0;
  virtual FUNCTION_DECL(assert_check_state, void) const = 0;
};


class TrafficLight : public TrafficController {
 public:
  TrafficLight(int phase_time,
               std::vector<SharedSignalGroup*> signal_groups);

  // Set all lights to red.
  FUNCTION_DECL(initialize, void);

  FUNCTION_DECL(step, void);

  // Make sure that only one group has green light.
  FUNCTION_DECL(assert_check_state, void) const;

 private:
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
  PriorityYieldTrafficController(std::vector<SharedSignalGroup*> groups)
      : groups_(groups) {}

  FUNCTION_DECL(initialize, void) {}

  FUNCTION_DECL(step, void);

  FUNCTION_DECL(assert_check_state, void) const;

 private:
  const std::vector<SharedSignalGroup*> groups_;

  // Check if a car is coming from this group within the next iteration.
  FUNCTION_DECL(has_incoming_traffic, bool, SharedSignalGroup* group) const;
  FUNCTION_DECL(has_incoming_traffic, bool, Cell* cell, int lookahead) const;
};


class Street {
 public:
  Street(Cell* first, Cell* last, Cell::Type type = Cell::kResidential)
      : first_(first), last_(last), type_(type) {
    assert(type >= 0 && type < Cell::kMaxType);
  }

  FUNCTION_DECL(first, Cell*) const { return first_; }
  FUNCTION_DECL(last, Cell*) const { return last_; }
  FUNCTION_DECL(type, Cell::Type) const { return type_; }

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
  FUNCTION_DECL(initialize, void);

  FUNCTION_DECL(random_cell, Cell*, uint32_t* state) const;

  FUNCTION_DECL(random_free_cell, Cell*, uint32_t* state) const;

  // Simulate a single tick.
  FUNCTION_DECL(step, void);

  FUNCTION_DECL(add_street, void, Street* street) {
    streets_.push_back(street);
  }

  FUNCTION_DECL(add_cell, void, Cell* cell) {
    cells_.push_back(cell);
  }
  
  FUNCTION_DECL(add_car, void, Car* car) {
    cars_.push_back(car);
  }

  FUNCTION_DECL(add_traffic_controller, void, TrafficController* light) {
    traffic_controllers_.push_back(light);
  }

  FUNCTION_DECL(add_inactive_car, void, Car* car) {
    inactive_cars_.push_back(car);
  }

  // Return a vector of all cars. Only used for debug output.
  FUNCTION_DECL(cars, const std::vector<Car*>&) const { return cars_; }

  // Print information about this simulation.
  FUNCTION_DECL(print_stats, void) const;

  // Calculate a checksum for the state of this simulation.
  FUNCTION_DECL(checksum, uint64_t) const;

 private:
  friend class Renderer;

  // A vector of all streets. Contains only the cells of start and
  // end points. Only used for GUI purposes.
  std::vector<Street*> streets_;
  FUNCTION_DECL(streets, const std::vector<Street*>&) const {
    return streets_;
  }

  // A vector of all cells.
  std::vector<Cell*> cells_;
  FUNCTION_DECL(cells, const std::vector<Cell*>&) const { return cells_; }

  // A vector of all cars.
  std::vector<Car*> cars_;

  // A vector of all traffic controllers, e.g., traffic lights.
  std::vector<TrafficController*> traffic_controllers_;

  // A vector of all inactive cars. Used to keep track of cars that are
  // leaving the map.
  std::vector<Car*> inactive_cars_;
};

#endif  // TRAFFIC_H
