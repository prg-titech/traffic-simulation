#include <cassert>

#include "random.h"

bool Cell::is_free() const {
#ifdef OPTION_STANDARD
  assert(is_free_ == (car_ == nullptr));
#endif  // OPTION_STANDARD
  return is_free_;
}

bool Car::is_jammed() const {
  if (path_.size() == 0) return false;

  assert(path_[0] != position_);
  return !path_[0]->is_free();
}

int Cell::max_velocity() const {
  if (controller_max_velocity_ < max_velocity_) {
    return controller_max_velocity_;
  } else {
    return max_velocity_;
  }
}

int Cell::street_max_velocity() const {
  return max_velocity_;
}

void Cell::occupy(PTR(Car) car) {
  assert(is_free_);
#ifdef OPTION_STANDARD
  assert(car_ == nullptr);
#endif  // OPTION_STANDARD

  is_free_ = false;
  car_ = car;

#ifdef OPTION_STANDARD
  draw();
#endif  // OPTION_STANDARD
}

void Cell::release() {
  assert(!is_free_);
#ifdef OPTION_STANDARD
  assert(car_ != nullptr);
  car_ = nullptr;
#endif  // OPTION_STANDARD

  is_free_ = true;

#ifdef OPTION_STANDARD
  draw();
#endif  // OPTION_STANDARD
}

PTR(Cell) Car::next_step(PTR(Cell) position) {
  // Random walk.
  auto cells = METHOD_CALL(Cell, position, outgoing_cells);
  assert(cells.size() > 0);

  PTR(Cell) next_cell = cells[rand32() % cells.size()];
  assert(next_cell != position);
  return next_cell;
}

void Car::step_velocity() {
  step_initialize_iteration();
  step_accelerate();
  step_extend_path();
  step_constraint_velocity();
}

void Car::step_move() {
  PTR(Cell) next_cell = position_;
  assert(velocity_ <= METHOD_CALL(Cell, next_cell, max_velocity));

  for (int i = 0; i < velocity_; ++i) {
    next_cell = path_.pop();
    assert(velocity_ <= METHOD_CALL(Cell, next_cell, max_velocity));
    assert(METHOD_CALL(Cell, next_cell, is_free));
  }

  METHOD_CALL(Cell, position_, release);
  METHOD_CALL(Cell, next_cell, occupy, THIS);
  position_ = next_cell;

  if (METHOD_CALL(Cell, position_, is_sink)) {
    // Remove car from the simulation.
    METHOD_CALL(Cell, position_, release);
    path_.shrink_to_size(0);
    set_active(false);

    // Add car at random cell.
    instance->add_inactive_car(THIS);
  }
}

void Car::step_initialize_iteration() {
  if (velocity_ == 0) {
    path_.shrink_to_size(0);
  }
}

void Car::step_accelerate() {
  if (velocity_ < max_velocity_) {
    ++velocity_;
  }

  assert(velocity_ <= max_velocity_);
  assert(velocity_ <= path_.capacity());
}

void Car::step_constraint_velocity() {
  // This is actually only needed for the very first iteration, because a car
  // may be positioned on a traffic light cell.
  if (velocity_ > METHOD_CALL(Cell, position_, max_velocity)) {
    velocity_ = METHOD_CALL(Cell, position_, max_velocity);
  }

  auto path_iter = path_.begin();
  int distance = 1;

  while (distance <= velocity_) {
    // Invariant: Movement of up to `distance - 1` many cells at `velocity_`
    //            is allowed.
    // Now check if next cell can be entered.
    PTR(Cell) next_cell = *path_iter;

    // Avoid collision.
    if (!METHOD_CALL(Cell, next_cell, is_free)) {
      // Cannot enter cell.
      --distance;
      velocity_ = distance;
      break;
    } // else: Can enter next cell.

    if (velocity_ > METHOD_CALL(Cell, next_cell, max_velocity)) {
      // Car is too fast for this cell.
      if (METHOD_CALL(Cell, next_cell, max_velocity) > distance - 1) {
        // Even if we slow down, we would still make progress.
        velocity_ = METHOD_CALL(Cell, next_cell, max_velocity);
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

void Car::step_extend_path() {
  assert(path_.capacity() >= velocity_);

  int num_steps = velocity_ - path_.size();
  PTR(Cell) position = position_;

  if (path_.size() > 0) {
    position = path_.back();
  }

  for (int i = 0; i < num_steps; ++i) {
    if (METHOD_CALL(Cell, position, is_sink)) {
      // End of map. Remove car from simulation here.
      velocity_ = path_.size();
      break;
    }

    position = next_step(position);
    path_.push(position);
  }

  assert(path_.size() >= velocity_);
}

void Car::set_position(Cell* cell) {
  path_.shrink_to_size(0);
  METHOD_CALL(Cell, cell, occupy, THIS);
  position_ = cell;
}

void Car::assert_check_velocity() const {
  assert(path_.size() >= velocity_);

  if (velocity_ > 0) {
    assert(path_[0] != position_);
  }

  for (int i = 0; i < velocity_; ++i) {
    assert(METHOD_CALL(Cell, path_[i], is_free));
    assert(velocity_ <= METHOD_CALL(Cell, path_[i], max_velocity));
  }
}

void Car::step_slow_down() {
  float rand_float = static_cast<float>(rand32())
      / static_cast<float>(RAND32_MAX);
  if (rand_float < 0.5 && velocity_ > 0) {
    --velocity_;
  }
}

uint32_t Car::rand32() {
  return ::rand32(&random_state_);
}

void SharedSignalGroup::signal_go() {
  for (auto it = cells_.begin(); it != cells_.end(); ++it) {
    METHOD_CALL(Cell, *it, remove_controller_max_velocity);
    assert(METHOD_CALL(Cell, *it, max_velocity) > 0);
  }
}

void SharedSignalGroup::signal_stop() {
  for (auto it = cells_.begin(); it != cells_.end(); ++it) {
    METHOD_CALL(Cell, *it, set_controller_max_velocity, 0);
  }
}

void TrafficLight::step() {
  timer_ = (timer_ + 1) % phase_time_;

  if (timer_ == 0) {
    METHOD_CALL(SharedSignalGroup, signal_groups_[phase_], signal_stop);
    phase_ = (phase_ + 1) % signal_groups_.size();
    METHOD_CALL(SharedSignalGroup, signal_groups_[phase_], signal_go);
  }
}

void TrafficLight::initialize() {
  for (auto it = signal_groups_.begin(); it != signal_groups_.end(); ++it) {
    METHOD_CALL(SharedSignalGroup, *it, signal_stop);
  }
}

void TrafficLight::assert_check_state() const {
#ifndef NDEBUG
  bool found_green = false;
  for (auto it = signal_groups_.begin(); it != signal_groups_.end(); ++it) {
    auto& group = (*it)->cells();

    for (auto it2 = group.begin(); it2 != group.end(); ++it2) {
      if (METHOD_CALL(Cell, *it2, max_velocity) > 0) {
        // Make sure only one group has a green light (or none).
        assert(!found_green);
        found_green = true;
        goto outer_loop_end;
      }
    }

    outer_loop_end:;
  }
#endif  // NDEBUG
}

void PriorityYieldTrafficController::assert_check_state() const {}

bool PriorityYieldTrafficController::has_incoming_traffic(
    PTR(Cell) cell, int lookahead) const {
  if (lookahead == 0) {
    return !METHOD_CALL(Cell, cell, is_free);
  }

  // Check incoming cells. This is BFS.
  for (int i = 0; i < cell->incoming_cells().size(); ++i) {
    if (has_incoming_traffic(cell->incoming_cells()[i], lookahead - 1)) {
      return true;
    }
  }

  return !METHOD_CALL(Cell, cell, is_free);
}

bool PriorityYieldTrafficController::has_incoming_traffic(
    PTR(SharedSignalGroup) group) const {
  const auto& cells = METHOD_CALL(SharedSignalGroup, group, cells);
  for (auto it = cells.begin(); it != cells.end(); ++it) {
    // Report incoming traffic if at least one cells in the group reports
    // incoming traffic.
    if (has_incoming_traffic(
        *it, METHOD_CALL(Cell, *it, street_max_velocity))) {
      return true;
    }
  }
  return false;
}

void PriorityYieldTrafficController::step() {
  bool found_traffic = false;
  // Cells are sorted by priority.
  for (int i = 0; i < groups_.size(); ++i) {
    bool has_incoming = has_incoming_traffic(groups_[i]);
    const auto& cells = METHOD_CALL(SharedSignalGroup, groups_[i], cells);

    if (!found_traffic && has_incoming) {
      found_traffic = true;
      // Allow traffic to flow.
      for (auto it = cells.begin(); it != cells.end(); ++it) {
        METHOD_CALL(Cell, *it, remove_controller_max_velocity);
      }
    } else if (has_incoming) {
      // Traffic with higher priority is incoming.
      for (auto it = cells.begin(); it != cells.end(); ++it) {
        METHOD_CALL(Cell, *it, set_controller_max_velocity, 0);
      }
    }
  }
}

void Simulation::step() {
  step_cells();
  step_traffic_controllers();
  step_cars();
}

void Simulation::step_cells() {
#ifndef NDEBUG
  // Make sure that no two cars are on the same cell.
  std::set<Cell*> occupied_cells;
  for (int i = 0; i < cars_.size(); ++i) {
    if (METHOD_CALL(Car, cars_[i], is_active)) {
      assert(occupied_cells.find(METHOD_CALL(Car, cars_[i], position))
             == occupied_cells.end());
      occupied_cells.insert(METHOD_CALL(Car, cars_[i], position));
    }
  }
#endif  // NDEBUG
}

void Simulation::step_cars() {
  for (int i = 0; i < cars_.size(); ++i) {
    if (cars_[i]->is_active())
        cars_[i]->step_velocity();
  }

#ifndef NDEBUG
  for (int i = 0; i < cars_.size(); ++i) {
    if (METHOD_CALL(Car, cars_[i], is_active))
        METHOD_CALL(Car, cars_[i], assert_check_velocity);
  }
#endif  // NDEBUG

  for (int i = 0; i < cars_.size(); ++i) {
    if (METHOD_CALL(Car, cars_[i], is_active))
        METHOD_CALL(Car, cars_[i], step_move);
  }

  // Reactivate cars.
  for (auto it = inactive_cars_.begin(); it != inactive_cars_.end(); ++it) {
    PTR(Cell) new_start_cell = random_free_cell(
        &METHOD_CALL(Car, *it, random_state));
    METHOD_CALL(Car, *it, set_position, new_start_cell);
    METHOD_CALL(Car, *it, set_active, true);
  }
  inactive_cars_.clear();
}

PTR(Cell) Simulation::random_cell(uint32_t* state) const {
  return cells_[rand32(state) % cells_.size()];
}

PTR(Cell) Simulation::random_free_cell(uint32_t* state) const {
  // Try max. of 100 times.
  for (int i = 0; i < 100; ++i) {
    int id = rand32(state) % cells_.size();
    if (METHOD_CALL(Cell, cells_[id], is_free)) {
      return cells_[id];
    }
  }

  // Could not find free cell.
  assert(false);
  return random_cell(state);
}

void Simulation::print_stats() const {
  cout << "Number of cells: " << cells_.size() << "\n"
       << "Number of cars: " << cars_.size() << "\n"
       << "Number of streets: " << streets_.size() << "\n"
       << "Number of traffic controllers: " << traffic_controllers_.size() 
       << "\n";
}

uint64_t Simulation::checksum() const {
  uint64_t c = 17;
  for (int i = 0; i < cars_.size(); ++i) {
    auto position = METHOD_CALL(Car, cars_[i], position);
    c += METHOD_CALL(Cell, position, x) + METHOD_CALL(Cell, position, y);
    c %= UINT64_MAX;
  }
  return c;
}
