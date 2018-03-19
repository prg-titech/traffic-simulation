#include <cassert>
#include <queue>
#include <utility>

#include "drawing.h"
#include "traffic.h"
#include "random.h"

using namespace std;

// Singleton simulation instance.
extern Simulation* simulation;

extern void draw_cell(const Cell& cell);

FUNCTION_DEF(Cell, Cell, , 
    int max_velocity, double x, double y, Type type, uint32_t tag)
    : is_free_(true), max_velocity_(max_velocity), type_(type),
      controller_max_velocity_(max_velocity), x_(x), y_(y), tag_(tag) {
  assert(type >= 0 && type < Type::kMaxType);
  assert(max_velocity_ > 0);
}

FUNCTION_DEF(Cell, connect_to, void, PTR(Cell) other) {
  assert(other != THIS);
  outgoing_cells_.push_back(other);
  other->incoming_cells_.push_back(THIS);
}

FUNCTION_DEF(Cell, draw, void) const {
  draw_cell(*THIS);
}

FUNCTION_DEF(Cell, is_free, bool) const {
  assert(is_free_ == (car_ == nullptr));
  return is_free_;
}

FUNCTION_DEF(Car, is_jammed, bool) const {
  if (path_.size() == 0) return false;

  assert(path_[0] != position_);
  return !path_[0]->is_free();
}

FUNCTION_DEF(Cell, max_velocity, int) const {
  if (controller_max_velocity_ < max_velocity_) {
    return controller_max_velocity_;
  } else {
    return max_velocity_;
  }
}

FUNCTION_DEF(Cell, street_max_velocity, int) const {
  return max_velocity_;
}

FUNCTION_DEF(Cell, occupy, void, PTR(Car) car) {
  assert(is_free_);
  assert(car_ == nullptr);

  is_free_ = false;
  car_ = car;
  draw();
}

FUNCTION_DEF(Cell, release, void) {
  assert(!is_free_);
  assert(car_ != nullptr);

  is_free_ = true;
  car_ = nullptr;
  draw();
}

FUNCTION_DEF(Car, next_step, PTR(Cell), PTR(Cell) position) {
  // Random walk.
  assert(position->outgoing_cells().size() > 0);
  auto& cells = position->outgoing_cells();

  PTR(Cell) next_cell = cells[rand32() % position->outgoing_cells().size()];
  assert(next_cell != position);
  return next_cell;
}

FUNCTION_DEF(Car, step_velocity, void) {
  step_initialize_iteration();
  step_accelerate();
  step_extend_path();
  step_constraint_velocity();
}

FUNCTION_DEF(Car, step_move, void) {
  PTR(Cell) next_cell = position_;
  assert(velocity_ <= next_cell->max_velocity());

  for (int i = 0; i < velocity_; ++i) {
    next_cell = path_.pop();
    assert(velocity_ <= next_cell->max_velocity());
    assert(next_cell->is_free());
  }

  position_->release();
  next_cell->occupy(THIS);
  position_ = next_cell;

  if (position_->is_sink()) {
    // Remove car from the simulation.
    position_->release();
    path_.shrink_to_size(0);
    set_active(false);

    // Add car at random cell.
    simulation->add_inactive_car(THIS);
  }
}

FUNCTION_DEF(Car, step_initialize_iteration, void) {
  if (velocity_ == 0) {
    path_.shrink_to_size(0);
  }
}

FUNCTION_DEF(Car, step_accelerate, void) {
  if (velocity_ < max_velocity_) {
    ++velocity_;
  }

  assert(velocity_ <= max_velocity_);
  assert(velocity_ <= path_.capacity());
}

FUNCTION_DEF(Car, step_constraint_velocity, void) {
  // This is actually only needed for the very first iteration, because a car
  // may be positioned on a traffic light cell.
  if (velocity_ > position_->max_velocity()) {
    velocity_ = position_->max_velocity();
  }

  auto path_iter = path_.begin();
  int distance = 1;

  while (distance <= velocity_) {
    // Invariant: Movement of up to `distance - 1` many cells at `velocity_`
    //            is allowed.
    // Now check if next cell can be entered.
    PTR(Cell) next_cell = *path_iter;

    // Avoid collision.
    if (!next_cell->is_free()) {
      // Cannot enter cell.
      --distance;
      velocity_ = distance;
      break;
    } // else: Can enter next cell.

    if (velocity_ > next_cell->max_velocity()) {
      // Car is too fast for this cell.
      if (next_cell->max_velocity() > distance - 1) {
        // Even if we slow down, we would still make progress.
        velocity_ = next_cell->max_velocity();
      } else {
        // Do not enter the next cell.
        --distance;
        velocity_ = distance;
        break;
      }
    }

    ++distance;
    ++path_iter;
  }

  --distance;
  assert(distance <= velocity_);
}

FUNCTION_DEF(Car, step_extend_path, void) {
  assert(path_.capacity() >= velocity_);

  int num_steps = velocity_ - path_.size();
  PTR(Cell) position = position_;

  if (path_.size() > 0) {
    position = path_.back();
  }

  for (int i = 0; i < num_steps; ++i) {
    if (position->is_sink()) {
      // End of map. Remove car from simulation here.
      velocity_ = path_.size();
      break;
    }

    position = next_step(position);
    path_.push(position);
  }

  assert(path_.size() >= velocity_);
}

FUNCTION_DEF(Car, set_position, void, PTR(Cell) cell) {
  path_.shrink_to_size(0);
  cell->occupy(THIS);
  position_ = cell;
}

FUNCTION_DEF(Car, assert_check_velocity, void) const {
  assert(path_.size() >= velocity_);

  if (velocity_ > 0) {
    assert(path_[0] != position_);
  }

  for (int i = 0; i < velocity_; ++i) {
    assert(path_[i]->is_free());
    assert(velocity_ <= path_[i]->max_velocity());
  }
}

FUNCTION_DEF(Car, step_slow_down, void) {
  float rand_float = static_cast<float>(rand32())
      / static_cast<float>(RAND32_MAX);
  if (rand_float < 0.5 && velocity_ > 0) {
    --velocity_;
  }
}

FUNCTION_DEF(Car, rand32, uint32_t) {
  return ::rand32(&random_state_);
}

FUNCTION_DEF(SharedSignalGroup, signal_go, void) {
  for (auto it = cells_.begin(); it != cells_.end(); ++it) {
    (*it)->remove_controller_max_velocity();
    assert((*it)->max_velocity() > 0);
  }
}

FUNCTION_DEF(SharedSignalGroup, signal_stop, void) {
  for (auto it = cells_.begin(); it != cells_.end(); ++it) {
    (*it)->set_controller_max_velocity(0);
  }
}

FUNCTION_DEF(TrafficLight, TrafficLight, ,
    int phase_time, std::vector<SharedSignalGroup*> signal_groups)
    : phase_time_(phase_time), signal_groups_(signal_groups) {
  // Initialize with random time.
  timer_ = rand() % phase_time_;
}

FUNCTION_DEF(TrafficLight, step, void) {
  timer_ = (timer_ + 1) % phase_time_;

  if (timer_ == 0) {
    signal_groups_[phase_]->signal_stop();
    phase_ = (phase_ + 1) % signal_groups_.size();
    signal_groups_[phase_]->signal_go();
  }
}

FUNCTION_DEF(TrafficLight, initialize, void) {
  for (auto it = signal_groups_.begin(); it != signal_groups_.end(); ++it) {
    (*it)->signal_stop();
  }
}

FUNCTION_DEF(TrafficLight, assert_check_state, void) const {
  bool found_green = false;
  for (auto it = signal_groups_.begin(); it != signal_groups_.end(); ++it) {
    auto& group = (*it)->cells();

    for (auto it2 = group.begin(); it2 != group.end(); ++it2) {
      if ((*it2)->max_velocity() > 0) {
        // Make sure only one group has a green light (or none).
        assert(!found_green);
        found_green = true;
        goto outer_loop_end;
      }
    }

    outer_loop_end:;
  }
}

FUNCTION_DEF(PriorityYieldTrafficController, assert_check_state, void) const {}

FUNCTION_DEF(PriorityYieldTrafficController, has_incoming_traffic, bool,
    PTR(Cell) cell, int lookahead) const {
  if (lookahead == 0) {
    return !cell->is_free();
  }

  // Check incoming cells. This is BFS.
  for (int i = 0; i < cell->incoming_cells().size(); ++i) {
    if (has_incoming_traffic(cell->incoming_cells()[i], lookahead - 1)) {
      return true;
    }
  }

  return !cell->is_free();
}

FUNCTION_DEF(PriorityYieldTrafficController, has_incoming_traffic, bool,
    SharedSignalGroup* group) const {
  for (auto it = group->cells().begin(); it != group->cells().end(); ++it) {
    // Report incoming traffic if at least one cells in the group reports
    // incoming traffic.
    if (has_incoming_traffic(*it, (*it)->street_max_velocity())) {
      return true;
    }
  }
  return false;
}

FUNCTION_DEF(PriorityYieldTrafficController, step, void) {
  bool found_traffic = false;
  // Cells are sorted by priority.
  for (int i = 0; i < groups_.size(); ++i) {
    bool has_incoming = has_incoming_traffic(groups_[i]);
    auto& cells = groups_[i]->cells();

    if (!found_traffic && has_incoming) {
      found_traffic = true;
      // Allow traffic to flow.
      for (auto it = cells.begin(); it != cells.end(); ++it) {
        (*it)->remove_controller_max_velocity();
      }
    } else if (has_incoming) {
      // Traffic with higher priority is incoming.
      for (auto it = cells.begin(); it != cells.end(); ++it) {
        (*it)->set_controller_max_velocity(0);
      }
    }
  }
}

FUNCTION_DEF(Simulation, step, void) {
#ifndef NDEBUG
  // Make sure that no two cars are on the same cell.
  std::set<PTR(Cell)> occupied_cells;
  for (int i = 0; i < cars_.size(); ++i) {
    if (cars_[i]->is_active()) {
      assert(occupied_cells.find(cars_[i]->position())
             == occupied_cells.end());
      occupied_cells.insert(cars_[i]->position());
    }
  }
#endif  // NDEBUG

  for (int i = 0; i < traffic_controllers_.size(); ++i) {
    traffic_controllers_[i]->step();
  }

#ifndef NDEBUG
  for (int i = 0; i < traffic_controllers_.size(); ++i) {
    traffic_controllers_[i]->assert_check_state();
  }
#endif  // NDEBUG

  for (int i = 0; i < cars_.size(); ++i) {
    if (cars_[i]->is_active())
        cars_[i]->step_velocity();
  }

#ifndef NDEBUG
  for (int i = 0; i < cars_.size(); ++i) {
    if (cars_[i]->is_active())
        cars_[i]->assert_check_velocity();
  }
#endif  // NDEBUG

  for (int i = 0; i < cars_.size(); ++i) {
    if (cars_[i]->is_active())
        cars_[i]->step_move();
  }

  // Reactivate cars.
  for (auto it = inactive_cars_.begin(); it != inactive_cars_.end(); ++it) {
    PTR(Cell) new_start_cell = random_free_cell(&((*it)->random_state_));
    (*it)->set_position(new_start_cell);
    (*it)->set_active(true);
  }
  inactive_cars_.clear();
}

FUNCTION_DEF(Simulation, random_cell, PTR(Cell), uint32_t* state) const {
  return cells_[rand32(state) % cells_.size()];
}

FUNCTION_DEF(Simulation, random_free_cell, PTR(Cell), uint32_t* state) const {
  // Try max. of 100 times.
  for (int i = 0; i < 100; ++i) {
    int id = rand32(state) % cells_.size();
    if (cells_[id]->is_free()) {
      return cells_[id];
    }
  }

  // Could not find free cell.
  assert(false);
  return random_cell(state);
}

FUNCTION_DEF(Simulation, print_stats, void) const {
  cout << "Number of cells: " << cells_.size() << "\n"
       << "Number of cars: " << cars_.size() << "\n"
       << "Number of streets: " << streets_.size() << "\n"
       << "Number of traffic controllers: " << traffic_controllers_.size() 
       << "\n";
}

FUNCTION_DEF(Simulation, checksum, uint64_t) const {
  uint64_t c = 17;
  for (int i = 0; i < cars_.size(); ++i) {
    c += cars_[i]->position()->x() + cars_[i]->position()->y();
    c %= UINT64_MAX;
  }
  return c;
}

FUNCTION_DEF(Simulation, initialize, void) {
  for (int i = 0; i < traffic_controllers_.size(); ++i) {
    traffic_controllers_[i]->initialize();
  }
}
