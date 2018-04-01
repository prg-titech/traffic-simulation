#ifndef TRAFFIC_AOS_INT_H
#define TRAFFIC_AOS_INT_H

#include <cassert>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <vector>
#include <set>

#include "span.h"
#include "traffic.h"

#include "option_aos_int.inc"
#include "fixed_size_queue.h"

namespace simulation {
namespace aos_int {

class Car;

// Cell for traffic flow simulation based on cellular automaton.
class Cell {
 public:
  using Type = simulation::standard::Cell::Type;
  static const uint32_t kTurnLane = 1;

  Cell(simulation::standard::Cell* cell);

  // Returns true if the cell is free.
  bool is_free() const;

  // Returns the maximum velocity allowed on this cell at this moment. This
  // function takes into account velocity limitations due to traffic lights.
  int max_velocity() const;

  // Return max. velocity regardless of traffic controllers.
  int street_max_velocity() const;

  // A car enters this cell.
  void occupy(IndexType car);

  // A car leaves this cell.
  void release();

  IndexType num_outgoing_cells() const;

  IndexType outgoing_cell(IndexType index) const;

  IndexType num_incoming_cells() const;

  IndexType incoming_cell(IndexType index) const;

  // Return x and y coordinates of this cell. For rendering purposes only.
  double x() const { return x_; }
  double y() const { return y_; }

  // Returns the type of this cell.
  Type type() const { return type_; }

  // Additional information can be stored in the tag.
  uint32_t tag() const { return tag_; }

  // Sets the maximum temporary speed limit (traffic controller).
  void set_controller_max_velocity(int velocity) {
    controller_max_velocity_ = velocity;
  }

  // Removes the maximum temporary speed limit.
  void remove_controller_max_velocity() {
    controller_max_velocity_ = max_velocity_;
  }

  // Returns the car that occupies this cell.
  IndexType car() const { return car_; }

  // Returns true if this cell is a sink.
  bool is_sink() const { return is_sink_; }

  IndexType id() const { return id_; }

 private:
  const IndexType id_;

  const Type type_;
  bool is_free_;
  const bool is_sink_;
  const int max_velocity_;
  int controller_max_velocity_;

  const IndexType num_incoming_cells_;
  const IndexType num_outgoing_cells_;
  const IndexType first_incoming_cell_idx_;
  const IndexType first_outgoing_cell_idx_;

  IndexType car_ = kMaxIndexType;

  // Coordinates on the map. Used for rendering.
  const double x_, y_;

  // Debug information only.
  uint32_t tag_;
};


class Car {
 public:
  Car(simulation::standard::Car* car);

  void assert_check_velocity() const;

  void step_velocity();

  void step_move();

  IndexType position() const { return position_; }

  void set_position(IndexType cell);

  bool is_active() const { return is_active_; }

  bool is_jammed() const;

  void set_active(bool is_active) { is_active_ = is_active; }

  int velocity() const { return velocity_; }

  // Path cannot be modified using this API. It can only be modified from
  // within this class.
  const fixed_size_queue<IndexType, false>& path() const { return path_; }

  IndexType id() const { return id_; }

 protected:
  friend class Simulation;

  // Assuming that the car is located at position, determine where to go next.
  IndexType next_step(IndexType position);

  void step_initialize_iteration();

  void step_accelerate();

  void step_extend_path();

  void step_constraint_velocity();

  void step_slow_down();

  void step_reactivate();

  IndexType random_free_cell() const;

 private:
  const IndexType id_;

  bool is_active_;
  int velocity_ = 0;
  int max_velocity_;

  // Maintain max_velocity_ many cells in the queue. This is the path that the
  // car is going to take. The maximum movement speed is limited by the
  // maximum velocity of every path cell.
  fixed_size_queue<IndexType, false> path_;

  IndexType position_;

  // Every car has a state for its random number generator.
  uint32_t random_state_;
  uint32_t rand32();
  uint32_t& random_state() { return random_state_; }
};


class SharedSignalGroup {
 public:
  SharedSignalGroup(simulation::standard::SharedSignalGroup* group);

  // Sets traffic lights to green.
  void signal_go();

  // Sets traffic lights to red.
  void signal_stop();

  IndexType num_cells() const;

  IndexType cell(IndexType index) const;

  IndexType id() const { return id_; }

 private:
  const IndexType id_;
  const IndexType num_cells_;
  const IndexType first_cell_idx_;
};


class TrafficController {
 public:
  // virtual void initialize() = 0;
  // virtual void step() = 0;
  // virtual void assert_check_state() const = 0;
};


class TrafficLight : public TrafficController {
 public:
  TrafficLight(simulation::standard::TrafficLight* light);

  // Set all lights to red.
  void initialize();

  void step();

  // Make sure that only one group has green light.
  void assert_check_state() const;

  IndexType id() const { return id_; }

 private:
  const IndexType id_;

  // This timer is increased with every step.
  int timer_;

  // The number of cycles in timer_ until the signal group is changed.
  const int phase_time_;

  // Index into groups_. The specified signal group has a green light.
  int phase_ = 0;

  // Cells which are set to "green" at the same time.
  const IndexType num_signal_groups_;
  const IndexType first_signal_group_idx_;

  IndexType num_signal_groups() const;
  IndexType signal_group(IndexType index) const;

  // Check if a car is coming from this group within the next iteration.
  bool has_incoming_traffic(IndexType group) const;
  bool has_incoming_traffic(IndexType cell, int lookahead) const;
};


class PriorityYieldTrafficController : public TrafficController {
 public:
  PriorityYieldTrafficController(
      simulation::standard::PriorityYieldTrafficController* controller);

  void initialize();

  void step();

  void assert_check_state() const;

  IndexType id() const { return id_; }

 private:
  const IndexType id_;

  const IndexType num_groups_;
  const IndexType first_group_idx_;

  IndexType num_groups() const;
  IndexType group(IndexType index) const;

  // Check if a car is coming from this group within the next iteration.
  bool has_incoming_traffic(IndexType group) const;
  bool has_incoming_traffic(IndexType cell, int lookahead) const;
};


class Simulation {
 public:
  Simulation(simulation::standard::Simulation* simulation);

  // Initialize this traffic simulation. May be called only when all streets
  // cars, traffic controllers, etc. were added.
  void initialize();

  IndexType random_cell();

  IndexType random_free_cell();

  // Simulate a single tick.
  void step();

  void add_inactive_car(IndexType car);

  // Print information about this simulation.
  void print_stats() const;

  // Calculate a checksum for the state of this simulation.
  uint64_t checksum() const;

  void print_velocity_histgram();

  // Accessor methods for cars.
  IndexType num_cars() const;
  IndexType car(IndexType index) const;

  uint32_t& random_state() { return random_state_; }

 private:
  friend class Car;

  void step_cells();
  void step_traffic_controllers();
  void step_cars();
  void step_random_state();

  IndexType num_cells() const;
  IndexType cell(IndexType index) const;

  // Every simulation has a state for its random number generator.
  uint32_t random_state_;
  uint32_t rand32();
};

}  // namespace aos_int
}  // namespace simulation

#endif  // TRAFFIC_H
