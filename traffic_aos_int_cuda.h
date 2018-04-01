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

#include "option_aos_int_cuda.inc"
#include "fixed_size_queue.h"

namespace simulation {
namespace aos_int_cuda {

class Car;

// Cell for traffic flow simulation based on cellular automaton.
class Cell {
 public:
  using Type = simulation::standard::Cell::Type;
  static const uint32_t kTurnLane = 1;

  // Returns true if the cell is free.
  __device__ bool is_free() const;

  // Returns the maximum velocity allowed on this cell at this moment. This
  // function takes into account velocity limitations due to traffic lights.
  __device__ int max_velocity() const;

  // Return max. velocity regardless of traffic controllers.
  __device__ int street_max_velocity() const;

  // A car enters this cell.
  __device__ void occupy(IndexType car);

  // A car leaves this cell.
  __device__ void release();

  __device__ IndexType num_outgoing_cells() const;

  __device__ IndexType outgoing_cell(IndexType index) const;

  __device__ IndexType num_incoming_cells() const;

  __device__ IndexType incoming_cell(IndexType index) const;

  // Return x and y coordinates of this cell. For rendering purposes only.
  __device__ double x() const { return x_; }
  __device__ double y() const { return y_; }

  // Returns the type of this cell.
  __device__ Type type() const { return type_; }

  // Additional information can be stored in the tag.
  __device__ uint32_t tag() const { return tag_; }

  // Sets the maximum temporary speed limit (traffic controller).
  __device__ void set_controller_max_velocity(int velocity) {
    controller_max_velocity_ = velocity;
  }

  // Removes the maximum temporary speed limit.
  __device__ void remove_controller_max_velocity() {
    controller_max_velocity_ = max_velocity_;
  }

  // Returns the car that occupies this cell.
  __device__ IndexType car() const { return car_; }

  // Returns true if this cell is a sink.
  __device__ bool is_sink() const { return is_sink_; }

  __device__ IndexType id() const { return id_; }

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
  __device__ void assert_check_velocity() const;

  __device__ void step_velocity();

  __device__ void step_move();

  __device__ void step_reactivate();

  __device__ IndexType position() const { return position_; }

  __device__ void set_position(IndexType cell);

  __device__ bool is_active() const { return is_active_; }

  __device__ bool is_jammed() const;

  __device__ void set_active(bool is_active) { is_active_ = is_active; }

  __device__ int velocity() const { return velocity_; }

  // Path cannot be modified using this API. It can only be modified from
  // within this class.
  __host__ __device__ const fixed_size_queue<IndexType, false>& path() const {
    return path_;
  }

  __device__ IndexType id() const { return id_; }

  __device__ uint32_t rand32();

  __device__ uint32_t& random_state() { return random_state_; }

 protected:
  friend class Simulation;

  // Assuming that the car is located at position, determine where to go next.
  __device__ IndexType next_step(IndexType position);

  __device__ void step_initialize_iteration();

  __device__ void step_accelerate();

  __device__ void step_extend_path();

  __device__ void step_constraint_velocity();

  __device__ void step_slow_down();

  __device__ IndexType random_free_cell() const;

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
};


class SharedSignalGroup {
 public:
  // Sets traffic lights to green.
  __device__ void signal_go();

  // Sets traffic lights to red.
  __device__ void signal_stop();

  __device__ IndexType num_cells() const;

  __device__ IndexType cell(IndexType index) const;

  __device__ IndexType id() const { return id_; }

 private:
  const IndexType id_;
  const IndexType num_cells_;
  const IndexType first_cell_idx_;
};


class TrafficController {
 public:
  // __device__ virtual void initialize() = 0;
  // __device__ virtual void step() = 0;
  // __device__ virtual void assert_check_state() const = 0;
};


class TrafficLight : public TrafficController {
 public:
  // Set all lights to red.
  __device__ void initialize();

  __device__ void step();

  // Make sure that only one group has green light.
  __device__ void assert_check_state() const;

  __device__ IndexType id() const { return id_; }

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

  __device__ IndexType num_signal_groups() const;
  __device__ IndexType signal_group(IndexType index) const;

  // Check if a car is coming from this group within the next iteration.
  __device__ bool has_incoming_traffic(IndexType group) const;
  __device__ bool has_incoming_traffic(IndexType cell, int lookahead) const;
};


class PriorityYieldTrafficController : public TrafficController {
 public:
  __device__ void initialize();

  __device__ void step();

  __device__ void assert_check_state() const;

  __device__ IndexType id() const { return id_; }

 private:
  const IndexType id_;

  const IndexType num_groups_;
  const IndexType first_group_idx_;

  __device__ IndexType num_groups() const;
  __device__ IndexType group(IndexType index) const;

  // Check if a car is coming from this group within the next iteration.
  __device__ bool has_incoming_traffic(IndexType group) const;
  __device__ bool has_incoming_traffic(IndexType cell, int lookahead) const;
};


class Simulation {
 public:
  // Initialize this traffic simulation. May be called only when all streets
  // cars, traffic controllers, etc. were added.
  __device__ void initialize();

  IndexType random_cell();

  IndexType random_free_cell();

  // Simulate a single tick.
  __device__ void step();

  __device__ void add_inactive_car(IndexType car);

  // Print information about this simulation.
  __device__ void print_stats() const;

  // Calculate a checksum for the state of this simulation.
  __device__ uint64_t checksum() const;

  __device__ void print_velocity_histgram();

  // Accessor methods for cars.
  __device__ IndexType num_cars() const;
  __device__ IndexType car(IndexType index) const;

  __device__ uint32_t& random_state() { return random_state_; }

  __device__ void step_random_state();

 private:
  friend class Car;

  __device__ void step_cells();
  __device__ void step_traffic_controllers();
  __device__ void step_cars();

  __device__ IndexType num_cells() const;
  __device__ IndexType cell(IndexType index) const;

  // Every simulation has a state for its random number generator.
  uint32_t random_state_;
  __device__ uint32_t rand32();
};

}  // namespace aos_int_cuda
}  // namespace simulation

#endif  // TRAFFIC_H
