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
  return !METHOD_CALL(Cell, path_[0], is_free);
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
  const IndexType num_outgoing =
      METHOD_CALL(Cell, position, num_outgoing_cells);
  assert(num_outgoing > 0);

  CPTR(Cell) next_cell =
      METHOD_CALL(Cell, position, outgoing_cell, rand32() % num_outgoing);
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

  const int num_steps = velocity_ - path_.size();
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

void Car::set_position(PTR(Cell) cell) {
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
  const float rand_float = static_cast<float>(rand32())
      / static_cast<float>(RAND32_MAX);
  if (rand_float < 0.5 && velocity_ > 0) {
    --velocity_;
  }
}

uint32_t Car::rand32() {
  return ::rand32(&random_state_);
}

void SharedSignalGroup::signal_go() {
  for (IndexType i = 0; i < num_cells(); ++i) {
    CPTR(Cell) next_cell = cell(i);
    METHOD_CALL(Cell, next_cell, remove_controller_max_velocity);
    assert(METHOD_CALL(Cell, next_cell, max_velocity) > 0);
  }
}

void SharedSignalGroup::signal_stop() {
  for (IndexType i = 0; i < num_cells(); ++i) {
    CPTR(Cell) next_cell = cell(i);
    METHOD_CALL(Cell, next_cell, set_controller_max_velocity, 0);
  }
}

void TrafficLight::step() {
  timer_ = (timer_ + 1) % phase_time_;

  if (timer_ == 0) {
    METHOD_CALL(SharedSignalGroup, signal_group(phase_), signal_stop);
    phase_ = (phase_ + 1) % num_signal_groups();
    METHOD_CALL(SharedSignalGroup, signal_group(phase_), signal_go);
  }
}

void TrafficLight::initialize() {
  for (IndexType i = 0; i < num_signal_groups(); ++i) {
    CPTR(SharedSignalGroup) next_group = signal_group(i);
    METHOD_CALL(SharedSignalGroup, next_group, signal_stop);
  }
}

void TrafficLight::assert_check_state() const {
#ifndef NDEBUG
  bool found_green = false;
  for (IndexType i = 0; i < num_signal_groups(); ++i) {
    CPTR(SharedSignalGroup) group = signal_group(i);
    const IndexType num_cells =
        METHOD_CALL(SharedSignalGroup, group, num_cells);

    for (IndexType j = 0; j < num_cells; ++j) {
      CPTR(Cell) cell = METHOD_CALL(SharedSignalGroup, group, cell, j);
      if (METHOD_CALL(Cell, cell, max_velocity) > 0) {
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
  const IndexType num_incoming = METHOD_CALL(Cell, cell, num_incoming_cells);
  for (IndexType i = 0; i < num_incoming; ++i) {
    CPTR(Cell) next_cell = METHOD_CALL(Cell, cell, incoming_cell, i);
    if (has_incoming_traffic(next_cell, lookahead - 1)) {
      return true;
    }
  }

  return !METHOD_CALL(Cell, cell, is_free);
}

bool PriorityYieldTrafficController::has_incoming_traffic(
    PTR(SharedSignalGroup) group) const {
  const IndexType num_cells = METHOD_CALL(SharedSignalGroup, group, num_cells);
  for (IndexType i = 0; i < num_cells; ++i) {
    CPTR(Cell) next_cell = METHOD_CALL(SharedSignalGroup, group, cell, i);

    // Report incoming traffic if at least one cells in the group reports
    // incoming traffic.
    if (has_incoming_traffic(
        next_cell, METHOD_CALL(Cell, next_cell, street_max_velocity))) {
      return true;
    }
  }
  return false;
}

void PriorityYieldTrafficController::step() {
  bool found_traffic = false;
  // Cells are sorted by priority.
  for (int i = 0; i < num_groups(); ++i) {
    CPTR(SharedSignalGroup) next_group = group(i);
    bool has_incoming = has_incoming_traffic(group(i));
    IndexType num_cells = METHOD_CALL(SharedSignalGroup,
                                      next_group, num_cells);

    if (!found_traffic && has_incoming) {
      found_traffic = true;
      // Allow traffic to flow.
      for (IndexType i = 0; i < num_cells; ++i) {
        METHOD_CALL(Cell, METHOD_CALL(SharedSignalGroup, next_group, cell, i),
                    remove_controller_max_velocity);
      }
    } else if (has_incoming) {
      // Traffic with higher priority is incoming.
      for (IndexType i = 0; i < num_cells; ++i) {
        METHOD_CALL(Cell, METHOD_CALL(SharedSignalGroup, next_group, cell, i),
                    set_controller_max_velocity, 0);
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
  std::set<PTR(Cell)> occupied_cells;
  for (IndexType i = 0; i < num_cars(); ++i) {
    if (METHOD_CALL(Car, car(i), is_active)) {
      assert(occupied_cells.find(METHOD_CALL(Car, car(i), position))
             == occupied_cells.end());
      occupied_cells.insert(METHOD_CALL(Car, car(i), position));
    }
  }
#endif  // NDEBUG
}

void Simulation::step_cars() {
  for (int i = 0; i < num_cars(); ++i) {
    if (METHOD_CALL(Car, car(i), is_active))
        METHOD_CALL(Car, car(i), step_velocity);
  }

#ifndef NDEBUG
  for (int i = 0; i < num_cars(); ++i) {
    if (METHOD_CALL(Car, car(i), is_active))
        METHOD_CALL(Car, car(i), assert_check_velocity);
  }
#endif  // NDEBUG

  for (int i = 0; i < num_cars(); ++i) {
    if (METHOD_CALL(Car, car(i), is_active))
        METHOD_CALL(Car, car(i), step_move);
  }

  reactivate_cars();
}

PTR(Cell) Simulation::random_cell(uint32_t* state) const {
  return cell(rand32(state) % num_cells());
}

PTR(Cell) Simulation::random_free_cell(uint32_t* state) const {
  // Try max. of 100 times.
  for (int i = 0; i < 100; ++i) {
    int id = rand32(state) % num_cells();
    if (METHOD_CALL(Cell, cell(id), is_free)) {
      return cell(id);
    }
  }

  // Could not find free cell.
  assert(false);
  return random_cell(state);
}

void Simulation::print_stats() const {
#ifdef OPTION_STANDARD
  cout << "Number of cells: " << num_cells() << "\n"
       << "Number of cars: " << num_cars() << "\n"
       << "Number of streets: " << streets_.size() << "\n"
       << "Number of traffic controllers: " << traffic_controllers_.size() 
       << "\n";
#endif  // OPTION_STANDARD
}

uint64_t Simulation::checksum() const {
  uint64_t c = 17;
  for (int i = 0; i < num_cars(); ++i) {
    auto position = METHOD_CALL(Car, car(i), position);
    c += METHOD_CALL(Cell, position, x) + METHOD_CALL(Cell, position, y);
    c %= UINT64_MAX;
  }
  return c;
}
