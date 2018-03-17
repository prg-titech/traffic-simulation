#include <cassert>
#include <queue>
#include <utility>

#include "drawing.h"
#include "traffic.h"

using namespace std;

// Singleton simulation instance.
extern Simulation* simulation;

extern void draw_cell(Cell* cell);

void Cell::draw() {
  draw_cell(this);
}

void Cell::occupy(Car* car) {
  assert(is_free_);
  assert(car_ == nullptr);

  is_free_ = false;
  car_ = car;
  draw();
}

void Cell::release() {
  assert(!is_free_);
  assert(car_ != nullptr);

  is_free_ = true;
  car_ = nullptr;
  draw();
}


void Car::step_velocity() {
  step_initialize_iteration();
  step_accelerate();
  step_extend_path();
  step_constraint_velocity();
}

void Car::step_move() {
  Cell* next_cell = position_;
  assert(velocity_ <= next_cell->max_velocity());

  for (int i = 0; i < velocity_; ++i) {
    next_cell = path_.pop();
    assert(velocity_ <= next_cell->max_velocity());
    assert(next_cell->is_free());
  }

  position_->release();
  next_cell->occupy(this);
  position_ = next_cell;

  if (position_->is_sink()) {
    // Remove car from the simulation.
    position_->release();
    path_.shrink_to_size(0);
    set_active(false);

    // Add car at random cell.
    simulation->add_inactive_car(this);
  }
}

Cell* Car::next_step(Cell* position) {
  // (Almost) Random walk.
  assert(position->num_outgoing_cells() > 0);
  auto** cells = position->outgoing_cells();

  /*
  if (position->num_outgoing_cells() == 2) {
    // Take the larger street with higher probability.
    if (cells[0]->type() > cells[1]->type()) {
      return cells[rand() % 1000 < 700 ? 0 : 1];
    } else if (cells[0]->type() < cells[1]->type()) {
      return cells[rand() % 1000 < 700 ? 1 : 0];
    }
  }
  */

  Cell* next_cell = cells[rand() % position->num_outgoing_cells()];
  assert(next_cell != position);
  return next_cell;
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
  if (velocity_ > position_->max_velocity()) {
    velocity_ = position_->max_velocity();
  }

  auto path_iter = path_.begin();
  int distance = 1;

  while (distance <= velocity_) {
    // Invariant: Movement of up to `distance - 1` many cells at `velocity_`
    //            is allowed.
    // Now check if next cell can be entered.
    Cell* next_cell = *path_iter;

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

void Car::step_extend_path() {
  assert(path_.capacity() >= velocity_);

  int num_steps = velocity_ - path_.size();
  Cell* position = position_;

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

void Car::set_position(Cell* cell) {
  path_.shrink_to_size(0);
  cell->occupy(this);
  position_ = cell;
}

void Car::assert_check_velocity() {
  assert(path_.size() >= velocity_);

  if (velocity_ > 0) {
    assert(path_[0] != position_);
  }

  for (int i = 0; i < velocity_; ++i) {
    assert(path_[i]->is_free());
    assert(velocity_ <= path_[i]->max_velocity());
  }
}

bool Car::is_jammed() {
  if (path_.size() == 0) return false;

  assert(path_[0] != position_);
  return !path_[0]->is_free();
}

void Car::step_slow_down() {
  float rand_float = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  if (rand_float < 0.5 && velocity_ > 0) {
    --velocity_;
  }
}


void SharedSignalGroup::signal_go() {
  for (auto it = cells_.begin(); it != cells_.end(); ++it) {
    (*it)->remove_controller_max_velocity();
    assert((*it)->max_velocity() > 0);
  }
}


void SharedSignalGroup::signal_stop() {
  for (auto it = cells_.begin(); it != cells_.end(); ++it) {
    (*it)->set_controller_max_velocity(0);
  }
}


void TrafficLight::step() {
  timer_ = (timer_ + 1) % phase_time_;

  if (timer_ == 0) {
    signal_groups_[phase_]->signal_stop();
    phase_ = (phase_ + 1) % signal_groups_.size();
    //phase_ = rand() % signal_groups_.size();
    signal_groups_[phase_]->signal_go();
  }
}


void TrafficLight::initialize() {
  for (auto it = signal_groups_.begin(); it != signal_groups_.end(); ++it) {
    (*it)->signal_stop();
  }
}


void TrafficLight::assert_check_state() {
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


void PriorityYieldTrafficController::assert_check_state() {}


bool PriorityYieldTrafficController::has_incoming_traffic(Cell* cell,
                                                          int lookahead) {
  if (lookahead == 0) {
    return !cell->is_free();
  }

  // Check incoming cells. This is BFS.
  for (int i = 0; i < cell->num_incoming_cells(); ++i) {
    if (has_incoming_traffic(cell->incoming_cells()[i], lookahead - 1)) {
      return true;
    }
  }

  return !cell->is_free();
}


bool PriorityYieldTrafficController::has_incoming_traffic(
    SharedSignalGroup* group) {
  for (auto it = group->cells().begin(); it != group->cells().end(); ++it) {
    // Report incoming traffic if at least one cells in the group reports
    // incoming traffic.
    if (has_incoming_traffic(*it, (*it)->street_max_velocity())) {
      return true;
    }
  }
  return false;
}


void PriorityYieldTrafficController::step() {
  int set_green = 0;
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


void Simulation::step() {
#ifndef NDEBUG
  // Make sure that no two cars are on the same cell.
  std::set<Cell*> occupied_cells;
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
    Cell* new_start_cell = random_free_cell();
    (*it)->set_position(new_start_cell);
    (*it)->set_active(true);
  }
  inactive_cars_.clear();
}


Cell* Simulation::random_cell() {
  return cells_[rand() % cells_.size()];
}


Cell* Simulation::random_free_cell() {
  // Try max. of 100 times.
  for (int i = 0; i < 100; ++i) {
    int id = rand() % cells_.size();
    if (cells_[id]->is_free()) {
      return cells_[id];
    }
  }

  // Could not find free cell.
  assert(false);
  return random_cell();
}


void Simulation::print_stats() {
  cout << "Number of cells: " << cells_.size() << "\n"
       << "Number of cars: " << cars_.size() << "\n"
       << "Number of streets: " << streets_.size() << "\n"
       << "Number of traffic controllers: " << traffic_controllers_.size() 
       << "\n";
}


void Simulation::initialize() {
  for (int i = 0; i < traffic_controllers_.size(); ++i) {
    traffic_controllers_[i]->initialize();
  }
}

