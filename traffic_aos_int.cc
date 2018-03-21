#include "traffic_aos_int.h"

namespace simulation {
namespace aos_int {

// Data storage.
Cell* s_cells;
IndexType s_size_cells = 0;

IndexType* s_outgoing_cells;
IndexType s_size_outgoing_cells = 0;

IndexType* s_incoming_cells;
IndexType s_size_incoming_cells = 0;

Car* s_cars;
IndexType s_size_cars = 0;

IndexType* s_inactive_cars;
IndexType s_size_inactive_cars = 0;

TrafficLight* s_traffic_lights;
IndexType s_size_traffic_lights = 0;

PriorityYieldTrafficController* s_priority_controllers;
IndexType s_size_priority_controllers = 0;

SharedSignalGroup* s_shared_signal_groups;
IndexType s_size_shared_signal_groups = 0;

ArraySpan<IndexType> allocate_outgoing_cells(IndexType size) {
  ArraySpan<IndexType> result(s_size_outgoing_cells, size);
  s_size_outgoing_cells += size;
  return result;
}

ArraySpan<IndexType> allocate_incoming_cells(IndexType size) {
  ArraySpan<IndexType> result(s_size_incoming_cells, size);
  s_size_incoming_cells += size;
  return result;
}

// Constructors / conversion.
Cell::Cell(simulation::standard::Cell* cell)
    : type_(cell->type_), is_free_(cell->is_free_), is_sink_(cell->is_sink_),
      max_velocity_(cell->max_velocity_),
      controller_max_velocity_(cell->controller_max_velocity_),
      x_(cell->x_), y_(cell->y_), tag_(cell->tag_) {
  outgoing_cells_ = allocate_outgoing_cells(cell->outgoing_cells_.size());
  incoming_cells_ = allocate_incoming_cells(cell->incoming_cells_.size());

  for (int i = 0; i < cell->outgoing_cells_.size(); ++i) {
    s_outgoing_cells[outgoing_cells_.start() + i] = 
        cell->outgoing_cells_[i]->id();
  }

  for (int i = 0; i < cell->incoming_cells_.size(); ++i) {
    s_incoming_cells[incoming_cells_.start() + i] = 
        cell->incoming_cells_[i]->id();
  }

  car_ = cell->car_->id();
}

Car::Car(simulation::standard::Car* car)
    : is_active_(car->is_active_), velocity_(car->velocity_),
      max_velocity_(car->max_velocity_), position_(car->position_->id()),
      random_state_(car->random_state_), path_(car->max_velocity) {
  assert(car->path_.size() == 0);
  // TODO: Path must be re-initialized on the GPU!
  // Or better: Concatenate ringbuffers!
}

SharedSignalGroup::SharedSignalGroup(
    simulation::standard::SharedSignalGroup* group) {
  cells_ = allocate_group_cells(group->cells_.size());
  for (int i = 0; i < group->cells_.size(); ++i) {
    cells_[i] = group->cells_[i]->id();
  }
}

TrafficLight::TrafficLight(simulation::standard::TrafficLight* light)
    : timer_(light->timer_), phase_time_(light->phase_time_),
      phase_(light->phase_) {
  signal_groups_ = allocate_signal_groups(light->signal_groups_.size());
  for (int i = 0; i < light->signal_groups_.size(); ++i) {
    signal_groups_[i] = light->signal_groups_[i]->id();
  }
}

PriorityYieldTrafficController::PriorityYieldTrafficController(
    simulation::standard::PriorityYieldTrafficController* controller) {
  groups_ = allocate_signal_groups(light->groups_.size());
  for (int i = 0; i < controller->groups_.size(); ++i) {
    groups_[i] = controller->groups_[i]->id();
  }
}

Simulation::Simulation(simulation::standard::Simulation* simulation) {
  // Create cells.
  s_size_cells = simulation->cells_.size();
  s_cells = new Cell[s_size_cells];

  // Create storage arrays for incoming/outgoing cells.
  IndexType num_outgoing_cells = 0;
  IndexType num_incoming_cells = 0;
  s_size_incoming_cells = s_size_outgoing_cells = 0;
  for (int i = 0; i < s_size_cells; ++i) {
    num_outgoing_cells += simulation->cells_[i]->outgoing_cells_.size();
    num_incoming_cells += simulation->cells_[i]->incoming_cells_.size();
  }
  s_outgoing_cells = new IndexType[num_outgoing_cells];
  s_incoming_cells = new IndexType[num_incoming_cells];

  for (int i = 0; i < s_size_cells; ++i) {
    s_cells[i] = Cell(simulation->cells_[i]);
  }

  // Create cars.
  s_size_cars = simulation->cars_.size();
  s_size_inactive_cars = 0;
  s_cars = new Car[s_size_cars];
  s_inactive_cars = new IndexType[s_size_cars];
  for (int i = 0; i < s_size_cars; ++i) {
    s_cars[i] = Car(simulation->cars_[i]);
  }

  // Count shared signal groups.
  s_size_shared_signal_groups = 0;
  for (int i = 0; simulation->traffic_controllers_.size(); ++i) {
    auto* next_ctrl = simulation->traffic_controllers_[i];
    if (auto* traffic_light =
        dynamic_cast<simulation::standard::TrafficLight*>(next_ctrl)) {
      s_size_shared_signal_groups += traffic_light->signal_groups_.size();
    } else if (auto* priority_ctrl =
        dynamic_cast<simulation::standard::PriorityYieldTrafficController*>(
            next_ctrl)) {
      s_size_shared_signal_groups += priority_ctrl->groups_.size();
    } else {
      printf("Cannot handle traffic controller type.\n");
      exit(1);
    }
  }
  s_shared_signal_groups = new SharedSignalGroup[s_size_shared_signal_groups];

  // Create traffic controllers.
  s_size_traffic_lights = 0;
  s_size_priority_controllers = 0;
  for (int i = 0; simulation->traffic_controllers_.size(); ++i) {
    auto* next_ctrl = simulation->traffic_controllers_[i];
    if (auto* traffic_light =
        dynamic_cast<simulation::standard::TrafficLight*>(next_ctrl)) {
      s_traffic_lights[s_size_traffic_lights++] = TrafficLight(traffic_light);
    } else if (auto* priority_ctrl =
        dynamic_cast<simulation::standard::PriorityYieldTrafficController*>(
            next_ctrl)) {
      s_priority_controllers[s_size_priority_controllers++] =
          PriorityYieldTrafficController(priority_ctrl)
    } else {
      printf("Cannot handle traffic controller type.\n");
      exit(1);
    }
  }
}

void Simulation::add_inactive_car(IndexType car) {
  inactive_cars_[s_size_inactive_cars++] = car;
}

// Logic for traffic flow simulation.
#include "traffic_logic.inc"

}  // namespace aos_int
}  // namespace simulation
