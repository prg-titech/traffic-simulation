#include "drawing.h"
#include "traffic.h"

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
  }

  position_->release();
  next_cell->occupy();
  position_ = next_cell;
}

Cell* Car::next_step(Cell* position) {
  // Random walk.
  assert(position->num_neighbors() > 0);
  return position->neighbors()[rand() % position->num_neighbors()];
}

void Car::step_accelerate() {
  if (velocity_ < max_velocity_) {
    ++velocity_;
  }
}

void Car::step_constraint_velocity() {
  int distance = 1;
  auto path_iter = path_.begin();
  bool is_done = false;

  while (distance < velocity_) {
    Cell* next_cell = *path_iter;

    // Avoid collision.
    if (!next_cell->is_free()) {
      velocity_ = distance - 1;
      is_done = true;
    } // else: Can enter next cell.

    if (next_cell->max_velocity() < velocity_) {
      // Car is too fast for this cell.
      if (next_cell->max_velocity() < distance - 1) {
        // Drive to the beginning of the cell with the current velocity,
        // but do not enter it.
        velocity_ = distance - 1;
        is_done = true;
      } else {
        // Apply speed limit (and possibly enter cell).
        velocity_ = next_cell->max_velocity();
      }
    }

    if (is_done) {
      break;
    } else {
      distance++;
      ++path_iter;
    }
  }
}

void Car::step_extend_path() {
  int num_steps = velocity_ - path_.size();
  Cell* position = position_;

  if (path_.size() > 0) {
    position = path_.back();
  }

  for (int i = 0; i < num_steps; ++i) {
    position = next_step(position);
    path_.push(position);
  }
}

void Car::step_slow_down() {
  if (rand() < 0.5 && velocity_ > 0) {
    --velocity_;
  }
}
