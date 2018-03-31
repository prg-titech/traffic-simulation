#include <cassert>

#include "random.h"

DEVICE_MODE bool Cell::is_free() const {
  assert(is_free_ == (car_ == PNULL));
  return is_free_;
}

DEVICE_MODE bool Car::is_jammed() const {
  if (path_.size() == 0) return false;

  assert(path_[0] != position_);
  return !METHOD_CALL(Cell, path_[0], is_free);
}

DEVICE_MODE int Cell::max_velocity() const {
  if (controller_max_velocity_ < max_velocity_) {
    return controller_max_velocity_;
  } else {
    return max_velocity_;
  }
}

DEVICE_MODE int Cell::street_max_velocity() const {
  return max_velocity_;
}

DEVICE_MODE void Cell::occupy(PTR(Car) car) {
  assert(is_free_);
  assert(car_ == PNULL);

  is_free_ = false;
  car_ = car;

#ifdef OPTION_STANDARD
  draw();
#endif  // OPTION_STANDARD
}

DEVICE_MODE void Cell::release() {
  assert(!is_free_);
  assert(car_ != PNULL);
  car_ = PNULL;
  is_free_ = true;

#ifdef OPTION_STANDARD
  draw();
#endif  // OPTION_STANDARD
}

DEVICE_MODE PTR(Cell) Car::next_step(PTR(Cell) position) {
  // Almost random walk.
  const IndexType num_outgoing =
      METHOD_CALL(Cell, position, num_outgoing_cells);
  assert(num_outgoing > 0);

  // Collect probabilities of outgoing cells.
  int prob_sum = 0;
  for (int i = 0; i < num_outgoing; ++i) {
    CPTR(Cell) candidate = METHOD_CALL(Cell, position, outgoing_cell, i);
    if (METHOD_CALL(Cell, candidate, type) >=
        METHOD_CALL(Cell, position, type)) {
      // More likely to drive onto larger roads.
      prob_sum += 20;
    } else {
      prob_sum += 10;
    }
  }

  // Now choose a path randomly.
  int rand_select = rand32() % prob_sum;
  int threshold = 0;
  for (int i = 0; i < num_outgoing; ++i) {
    CPTR(Cell) candidate = METHOD_CALL(Cell, position, outgoing_cell, i);
    if (METHOD_CALL(Cell, candidate, type) >=
        METHOD_CALL(Cell, position, type)) {
      threshold += 20;
    } else {
      threshold += 10;
    }

    if (rand_select < threshold) {
      assert(candidate != position);
      return candidate;
    }
  }

  // We should never end up here.
  assert(false);
}

DEVICE_MODE void Car::step_velocity() {
  step_initialize_iteration();
  step_accelerate();
  step_extend_path();
  step_constraint_velocity();
}

DEVICE_MODE void Car::step_move() {
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
    // Remove car from the simulation. Will be added again in the next
    // iteration.
    METHOD_CALL(Cell, position_, release);
    path_.shrink_to_size(0);
    set_active(false);
  }
}

DEVICE_MODE void Car::step_initialize_iteration() {
  if (velocity_ == 0) {
    path_.shrink_to_size(0);
  }
}

DEVICE_MODE void Car::step_accelerate() {
  if (velocity_ < max_velocity_) {
    ++velocity_;
  }

  assert(velocity_ <= max_velocity_);
  assert(velocity_ <= path_.capacity());
}

DEVICE_MODE void Car::step_constraint_velocity() {
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

DEVICE_MODE void Car::step_extend_path() {
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

DEVICE_MODE void Car::set_position(PTR(Cell) cell) {
  path_.shrink_to_size(0);
  METHOD_CALL(Cell, cell, occupy, THIS);
  position_ = cell;
}

DEVICE_MODE void Car::assert_check_velocity() const {
  assert(path_.size() >= velocity_);

  if (velocity_ > 0) {
    assert(path_[0] != position_);
  }

  for (int i = 0; i < velocity_; ++i) {
    assert(METHOD_CALL(Cell, path_[i], is_free));
    assert(velocity_ <= METHOD_CALL(Cell, path_[i], max_velocity));
  }
}

DEVICE_MODE void Car::step_slow_down() {
  const float rand_float = static_cast<float>(rand32())
      / static_cast<float>(RAND32_MAX);
  if (rand_float < 0.5 && velocity_ > 0) {
    --velocity_;
  }
}

DEVICE_MODE PTR(Cell) Car::random_free_cell() const {
  uint64_t state = instance->random_state();
  uint64_t num_cars = instance->num_cars();
  uint64_t num_cells = instance->num_cells();
  uint32_t max_tries = num_cells / num_cars;

  // Try max. of 100 times.
  for (uint32_t i = 0; i < max_tries; ++i) {
    uint64_t cell_id = (num_cells * (id() + state) / num_cars + i)
                       % num_cells;
    PTR(Cell) next_cell = instance->cell(cell_id);
    if (METHOD_CALL(Cell, next_cell, is_free)) {
      return next_cell;
    }
  }

  // Could not find free cell. Try again in next iteration.
  return PNULL;
}

DEVICE_MODE void Car::step_reactivate() {
  // Set to random position if car is not active.
  if (!is_active_) {
    PTR(Cell) free_cell = random_free_cell();

    if (free_cell != PNULL) {
      set_position(free_cell);
      set_active(true);
    }
  }
}

DEVICE_MODE uint32_t Car::rand32() {
  return ::rand32(&random_state_);
}

DEVICE_MODE uint32_t Simulation::rand32() {
  return ::rand32(&random_state_);
}

DEVICE_MODE void SharedSignalGroup::signal_go() {
  for (IndexType i = 0; i < num_cells(); ++i) {
    CPTR(Cell) next_cell = cell(i);
    METHOD_CALL(Cell, next_cell, remove_controller_max_velocity);
    assert(METHOD_CALL(Cell, next_cell, max_velocity) > 0);
  }
}

DEVICE_MODE void SharedSignalGroup::signal_stop() {
  for (IndexType i = 0; i < num_cells(); ++i) {
    CPTR(Cell) next_cell = cell(i);
    METHOD_CALL(Cell, next_cell, set_controller_max_velocity, 0);
  }
}

DEVICE_MODE void TrafficLight::step() {
  timer_ = (timer_ + 1) % phase_time_;

  if (timer_ == 0) {
    METHOD_CALL(SharedSignalGroup, signal_group(phase_), signal_stop);
    phase_ = (phase_ + 1) % num_signal_groups();
    METHOD_CALL(SharedSignalGroup, signal_group(phase_), signal_go);
  }
}

DEVICE_MODE void TrafficLight::initialize() {
  for (IndexType i = 0; i < num_signal_groups(); ++i) {
    CPTR(SharedSignalGroup) next_group = signal_group(i);
    METHOD_CALL(SharedSignalGroup, next_group, signal_stop);
  }
}

DEVICE_MODE void PriorityYieldTrafficController::initialize() {
  for (IndexType i = 0; i < num_groups(); ++i) {
    CPTR(SharedSignalGroup) next_group = group(i);
    METHOD_CALL(SharedSignalGroup, next_group, signal_stop);
  }
}

DEVICE_MODE void TrafficLight::assert_check_state() const {
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

DEVICE_MODE void PriorityYieldTrafficController::assert_check_state() const {}

DEVICE_MODE bool PriorityYieldTrafficController::has_incoming_traffic(
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

DEVICE_MODE bool PriorityYieldTrafficController::has_incoming_traffic(
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

DEVICE_MODE void PriorityYieldTrafficController::step() {
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

DEVICE_MODE void Simulation::step_random_state() {
  rand32();
}

DEVICE_MODE void Simulation::step() {
  step_random_state();
  step_cells();
  step_traffic_controllers();
  step_cars();
}

DEVICE_MODE void Simulation::step_cells() {
#ifndef NDEBUG
#ifdef OPTION_STANDARD
  // Make sure that no two cars are on the same cell.
  std::set<PTR(Cell)> occupied_cells;
  for (IndexType i = 0; i < num_cars(); ++i) {
    if (METHOD_CALL(Car, car(i), is_active)) {
      assert(occupied_cells.find(METHOD_CALL(Car, car(i), position))
             == occupied_cells.end());
      occupied_cells.insert(METHOD_CALL(Car, car(i), position));
    }
  }
#endif  // OPTION_STANDARD
#endif  // NDEBUG
}

DEVICE_MODE void Simulation::step_cars() {
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

  for (int i = 0; i < num_cars(); ++i) {
    if (!METHOD_CALL(Car, car(i), is_active))
        METHOD_CALL(Car, car(i), step_reactivate);
  }
}

// TODO: These functions are only required in host mode!
DEVICE_MODE PTR(Cell) Simulation::random_cell() {
  return cell(rand32() % num_cells());
}

DEVICE_MODE PTR(Cell) Simulation::random_free_cell() {
  // Try max. of 100 times.
  for (int i = 0; i < 100; ++i) {
    int id = rand32() % num_cells();
    if (METHOD_CALL(Cell, cell(id), is_free)) {
      return cell(id);
    }
  }

  // Could not find free cell.
  assert(false);
  return random_cell();
}

DEVICE_MODE void Simulation::print_stats() const {
#ifdef OPTION_STANDARD
  cout << "Number of cells: " << num_cells() << "\n"
       << "Number of cars: " << num_cars() << "\n"
       << "Number of streets: " << streets_.size() << "\n"
       << "Number of traffic controllers: " << traffic_controllers_.size() 
       << "\n";
#endif  // OPTION_STANDARD
}

DEVICE_MODE uint64_t Simulation::checksum() const {
  uint64_t c = 17;
  for (int i = 0; i < num_cars(); ++i) {
    auto position = METHOD_CALL(Car, car(i), position);
    int tmp = METHOD_CALL(Cell, position, x) * METHOD_CALL(Cell, position, y) * METHOD_CALL(Car, car(i), id);
    c += tmp % 139;
  }
  return c;
}

DEVICE_MODE void Simulation::print_velocity_histgram() {
  int counter[100];
  for (int i = 0; i < 100; ++i) {
    counter[i] = 0;
  }

  for (int i = 0; i < num_cars(); ++i) {
    counter[METHOD_CALL(Car, car(i), velocity)]++;
  }

  for (int i = 0; i < 100; ++i) {
    printf("velocity[%i] = %i\n", i, counter[i]);
  }  
}
