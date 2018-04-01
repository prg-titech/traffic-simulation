#include "traffic_aos_int.h"
#include "random.h"

using namespace std;

namespace simulation {
namespace aos_int {

// Singleton simulation instance.
extern Simulation* instance;

// Data storage.
Cell* s_Cell;
IndexType s_size_Cell = 0;

IndexType* s_outgoing_cells;
IndexType s_size_outgoing_cells = 0;

IndexType* s_incoming_cells;
IndexType s_size_incoming_cells = 0;

Car* s_Car;
IndexType s_size_Car = 0;

IndexType* s_car_paths;
IndexType s_size_car_paths = 0;

IndexType* s_inactive_cars;
IndexType s_size_inactive_cars = 0;

TrafficLight* s_TrafficLight;
IndexType s_size_TrafficLight = 0;

PriorityYieldTrafficController* s_PriorityYieldTrafficController;
IndexType s_size_PriorityYieldTrafficController = 0;

SharedSignalGroup* s_SharedSignalGroup;
IndexType s_size_SharedSignalGroup = 0;

IndexType* s_traffic_light_signal_groups;
IndexType s_size_traffic_light_signal_groups = 0;

IndexType* s_priority_ctrl_signal_groups;
IndexType s_size_priority_ctrl_signal_groups = 0;

IndexType* s_signal_group_cells;
IndexType s_size_signal_group_cells = 0;

// Constructors / conversion.
Cell::Cell(simulation::standard::Cell* cell)
    : type_(cell->type_), is_free_(cell->is_free_), is_sink_(cell->is_sink_),
      max_velocity_(cell->max_velocity_), id_(cell->id_),
      controller_max_velocity_(cell->controller_max_velocity_),
      x_(cell->x_), y_(cell->y_), tag_(cell->tag_),
      num_outgoing_cells_(cell->outgoing_cells_.size()),
      num_incoming_cells_(cell->incoming_cells_.size()),
      first_outgoing_cell_idx_(s_size_outgoing_cells), 
      first_incoming_cell_idx_(s_size_incoming_cells) {
  s_size_outgoing_cells += cell->outgoing_cells_.size();
  s_size_incoming_cells += cell->incoming_cells_.size();

  for (int i = 0; i < cell->outgoing_cells_.size(); ++i) {
    s_outgoing_cells[first_outgoing_cell_idx_ + i] =
        cell->outgoing_cells_[i]->id();
    assert(cell->outgoing_cells_[i]->id() != cell->id());
  }

  for (int i = 0; i < cell->incoming_cells_.size(); ++i) {
    s_incoming_cells[first_incoming_cell_idx_ + i] = 
        cell->incoming_cells_[i]->id();
  }

  if (cell->car_ != nullptr) {
    car_ = cell->car_->id();
  } else {
    car_ = kMaxIndexType;
  }
}

Car::Car(simulation::standard::Car* car)
    : is_active_(car->is_active_), velocity_(car->velocity_), id_(car->id_),
      max_velocity_(car->max_velocity_), position_(car->position_->id()),
      random_state_(car->random_state_),
      path_(&s_car_paths[s_size_car_paths], car->max_velocity_) {
  assert(car->max_velocity_ == path_.capacity());
  assert(car->path_.size() == 0);
  s_size_car_paths += car->max_velocity_ + 1;
}

SharedSignalGroup::SharedSignalGroup(
    simulation::standard::SharedSignalGroup* group)
    : id_(group->id_), num_cells_(group->cells_.size()),
      first_cell_idx_(s_size_signal_group_cells) {
  s_size_signal_group_cells += group->cells_.size();
  for (int i = 0; i < group->cells_.size(); ++i) {
    s_signal_group_cells[first_cell_idx_ + i] = group->cells_[i]->id();
  }
}

TrafficLight::TrafficLight(simulation::standard::TrafficLight* light)
    : timer_(light->timer_), phase_time_(light->phase_time_),
      phase_(light->phase_), id_(light->id_),
      num_signal_groups_(light->signal_groups_.size()),
      first_signal_group_idx_(s_size_traffic_light_signal_groups) {
  s_size_traffic_light_signal_groups += light->signal_groups_.size();
  for (int i = 0; i < light->signal_groups_.size(); ++i) {
    auto group_id = light->signal_groups_[i]->id();
    s_traffic_light_signal_groups[first_signal_group_idx_ + i] = group_id;
    new(&s_SharedSignalGroup[group_id])
        SharedSignalGroup(light->signal_groups_[i]);
  }
}

PriorityYieldTrafficController::PriorityYieldTrafficController(
    simulation::standard::PriorityYieldTrafficController* controller)
        : id_(controller->id_), num_groups_(controller->groups_.size()),
          first_group_idx_(s_size_priority_ctrl_signal_groups) {
  s_size_priority_ctrl_signal_groups += controller->groups_.size();
  for (int i = 0; i < controller->groups_.size(); ++i) {
    auto group_id = controller->groups_[i]->id();
    s_priority_ctrl_signal_groups[first_group_idx_ + i] = group_id;
    new(&s_SharedSignalGroup[group_id])
        SharedSignalGroup(controller->groups_[i]);
  }
}

Simulation::Simulation(simulation::standard::Simulation* simulation)
    : random_state_(simulation->random_state_) {
  printf("Converting to AOS_INT...\n");
  fflush(stdout);

  // Create cells.
  s_size_Cell = simulation->num_cells();
  s_Cell = (Cell*) new char[s_size_Cell * sizeof(Cell)];

  // Create storage arrays for incoming/outgoing cells.
  IndexType num_outgoing_cells = 0;
  IndexType num_incoming_cells = 0;
  s_size_incoming_cells = s_size_outgoing_cells = 0;
  for (int i = 0; i < s_size_Cell; ++i) {
    num_outgoing_cells += simulation->cells_[i]->outgoing_cells_.size();
    num_incoming_cells += simulation->cells_[i]->incoming_cells_.size();
  }
  s_outgoing_cells = new IndexType[num_outgoing_cells];
  s_incoming_cells = new IndexType[num_incoming_cells];

  for (int i = 0; i < s_size_Cell; ++i) {
    new(&s_Cell[i]) Cell(simulation->cells_[i]);
  }

  printf("AOS_INT: Created cells.\n");
  fflush(stdout);

  // Determine length of global path array.
  IndexType num_path_array = 0;
  s_size_car_paths = 0;
  for (int i = 0; i < simulation->cars_.size(); ++i) {
    // path_ buffer must be one larger than max_velocity_ for iterator.
    num_path_array += 1 + simulation->cars_[i]->max_velocity_;
  }
  s_car_paths = new IndexType[num_path_array];

  // Create cars.
  s_size_Car = simulation->cars_.size();
  s_size_inactive_cars = 0;
  s_Car = (Car*) new char[s_size_Car * sizeof(Car)];
  s_inactive_cars = new IndexType[s_size_Car];
  for (int i = 0; i < s_size_Car; ++i) {
    new(&s_Car[i]) Car(simulation->cars_[i]);
  }

  printf("AOS_INT: Created cars.\n");
  fflush(stdout);

  // Count shared signal groups. And their combined size (#cells).
  s_size_SharedSignalGroup = 0;
  s_size_TrafficLight = 0;
  s_size_PriorityYieldTrafficController = 0;
  IndexType num_signal_group_cells = 0;
  IndexType num_traffic_light_signal_groups = 0;
  IndexType num_priority_ctrl_signal_groups = 0;
  IndexType num_traffic_lights = 0;
  IndexType num_priority_ctrls = 0;
  for (int i = 0; i < simulation->traffic_controllers_.size(); ++i) {
    auto* next_ctrl = simulation->traffic_controllers_[i];
    if (auto* traffic_light =
        dynamic_cast<simulation::standard::TrafficLight*>(next_ctrl)) {
      ++num_traffic_lights;
      num_traffic_light_signal_groups += traffic_light->signal_groups_.size();
      for (int j = 0; j < traffic_light->signal_groups_.size(); ++j) {
        num_signal_group_cells += 
            traffic_light->signal_groups_[j]->cells_.size();
      }
    } else if (auto* priority_ctrl =
        dynamic_cast<simulation::standard::PriorityYieldTrafficController*>(
            next_ctrl)) {
      ++num_priority_ctrls;
      num_priority_ctrl_signal_groups += priority_ctrl->groups_.size();
      for (int j = 0; j < priority_ctrl->groups_.size(); ++j) {
        num_signal_group_cells += priority_ctrl->groups_[j]->cells_.size();
      }
    } else {
      printf("Cannot handle traffic controller type.\n");
      exit(1);
    }
  }

  s_size_SharedSignalGroup = num_traffic_light_signal_groups +
                             num_priority_ctrl_signal_groups;
  s_SharedSignalGroup = (SharedSignalGroup*) new char[sizeof(SharedSignalGroup)
      * s_size_SharedSignalGroup];
  s_traffic_light_signal_groups =
      new IndexType[num_traffic_light_signal_groups];
  s_priority_ctrl_signal_groups =
      new IndexType[num_priority_ctrl_signal_groups];
  s_signal_group_cells = new IndexType[num_signal_group_cells];
  s_TrafficLight = (TrafficLight*) new char[
      num_traffic_lights * sizeof(TrafficLight)];
  s_PriorityYieldTrafficController = (PriorityYieldTrafficController*)
      new char[num_priority_ctrls * sizeof(PriorityYieldTrafficController)];

  // Create traffic controllers.
  for (int i = 0; i < simulation->traffic_controllers_.size(); ++i) {
    auto* next_ctrl = simulation->traffic_controllers_[i];
    if (auto* traffic_light =
        dynamic_cast<simulation::standard::TrafficLight*>(next_ctrl)) {
      new(&s_TrafficLight[s_size_TrafficLight++]) TrafficLight(traffic_light);
    } else if (auto* priority_ctrl =
        dynamic_cast<simulation::standard::PriorityYieldTrafficController*>(
            next_ctrl)) {
      new(&s_PriorityYieldTrafficController[
          s_size_PriorityYieldTrafficController++])
              PriorityYieldTrafficController(priority_ctrl);
    } else {
      printf("Cannot handle traffic controller type.\n");
      exit(1);
    }
  }

  printf("Converted simulation to AOS_INT.\n");
  fflush(stdout);
}

void Simulation::add_inactive_car(IndexType car) {
  s_inactive_cars[s_size_inactive_cars++] = car;
}

void Simulation::step_traffic_controllers() {
  //printf("NUM LIGHTS: %i\n", s_size_TrafficLight);
  for (int i = 0; i < s_size_TrafficLight; ++i) {
    s_TrafficLight[i].step();
  }

  for (int i = 0; i < s_size_PriorityYieldTrafficController; ++i) {
    s_PriorityYieldTrafficController[i].step();
  }
}

IndexType Cell::num_outgoing_cells() const {
  return num_outgoing_cells_;
}

IndexType Cell::outgoing_cell(IndexType index) const {
  return s_outgoing_cells[first_outgoing_cell_idx_ + index];
}

IndexType Cell::num_incoming_cells() const {
  return num_incoming_cells_;
}

IndexType Cell::incoming_cell(IndexType index) const {
  return s_incoming_cells[first_incoming_cell_idx_ + index];
}

IndexType SharedSignalGroup::num_cells() const { return num_cells_; }

IndexType SharedSignalGroup::cell(IndexType index) const {
  return s_signal_group_cells[first_cell_idx_ + index];
}

IndexType TrafficLight::num_signal_groups() const {
  return num_signal_groups_;
}

IndexType TrafficLight::signal_group(IndexType index) const {
  return s_traffic_light_signal_groups[first_signal_group_idx_ + index];
}

IndexType PriorityYieldTrafficController::num_groups() const {
  return num_groups_;
}

IndexType PriorityYieldTrafficController::group(IndexType index) const {
  return s_priority_ctrl_signal_groups[first_group_idx_ + index];
}

// Accessor methods for cars.
IndexType Simulation::num_cars() const { return s_size_Car; }
IndexType Simulation::car(IndexType index) const { return index; }

// Accessor methods for cells.
IndexType Simulation::num_cells() const { return s_size_Cell; }
IndexType Simulation::cell(IndexType index) const { return index; }

// Logic for traffic flow simulation.
#include "traffic_logic.inc"
#include "option_undo.inc"

}  // namespace aos_int
}  // namespace simulation
