#include <cassert>

#include "drawing.h"
#include "traffic.h"

extern void draw_cell(Cell* cell);

void Cell::draw() {
  draw_cell(this);
}

void Cell::occupy() {
  is_free_ = false;
  draw();
}

void Cell::release() {
  is_free_ = true;
  draw();
}


void Car::step_velocity() {
  step_accelerate();
  step_extend_path();
  step_constraint_velocity();
}

void Car::step_move() {
  Cell* next_cell = position_;
  for (int i = 0; i < velocity_; ++i) {
    next_cell = path_.pop();
    assert(velocity_ <= next_cell->max_velocity());
  }

  position_->release();
  next_cell->occupy();
  position_ = next_cell;
}

Cell* Car::next_step(Cell* position) {
  // Random walk.
  assert(position->num_outgoing_cells() > 0);
  return position->outgoing_cells()[rand() % position->num_outgoing_cells()];
}

void Car::step_accelerate() {
  if (velocity_ < max_velocity_) {
    ++velocity_;
  }

  assert(velocity_ <= max_velocity_);
  assert(velocity_ <= path_.capacity());
}

void Car::step_constraint_velocity() {
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
    position = next_step(position);
    path_.push(position);
  }

  assert(path_.size() >= velocity_);
}

void Car::step_slow_down() {
  float rand_float = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  if (rand_float < 0.5 && velocity_ > 0) {
    --velocity_;
  }
}


void SharedSignalGroup::signal_go() {
  for (auto it = cells_.begin(); it < cells_.end(); ++it) {
    (*it)->remove_controller_max_velocity();
  }
}


void SharedSignalGroup::signal_stop() {
  for (auto it = cells_.begin(); it < cells_.end(); ++it) {
    (*it)->set_controller_max_velocity(0);
  }
}


void TrafficLight::step() {
  timer_ = (timer_ + 1) % phase_time_;

  if (timer_ == 0) {
    signal_groups_[phase_]->signal_stop();
    phase_ = (phase_ + 1) % signal_groups_.size();
    signal_groups_[phase_]->signal_go();
  }
}


void TrafficLight::initialize() {
  for (auto it = signal_groups_.begin(); it != signal_groups_.end(); ++it) {
    (*it)->signal_stop();
  }
}


bool PriorityYieldTrafficController::has_incoming_traffic(Cell* cell,
                                                          int lookahead) {
  if (lookahead == 0) {
    // Don't care.
    return false;
  } else if (lookahead == 1) { 
    return !cell->is_free();
  }

  // Check incoming cells. This is BFS.
  bool result = false;
  for (int i = 0; i < cell->num_incoming_cells(); ++i) {
    result |= has_incoming_traffic(cell->incoming_cells()[i], lookahead - 1);
  }
  return result;
}


bool PriorityYieldTrafficController::has_incoming_traffic(Cell* cell) {
  return has_incoming_traffic(cell, cell->street_max_velocity());
}


void PriorityYieldTrafficController::step() {
  bool found_traffic = false;
  for (int i = 0; i < cells_.size(); ++i) {
    if (!found_traffic && has_incoming_traffic(cells_[i])) {
      found_traffic = true;
      // Allow traffic to flow.
      cells_[i]->remove_controller_max_velocity();
      continue;
    } else if (found_traffic) {
      // Traffic with higher priority is incoming.
      cells_[i]->set_controller_max_velocity(0);
    }
  }
}

