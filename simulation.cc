#include <chrono>
#include <thread>

#include "drawing.h"
#include "traffic.h"
#include "simple_network_builder.h"

int num_cells;
Cell** cells;

int num_cars = 200;
Car** cars;

int num_traffic_lights;
TrafficLight** lights;

void init_network() {
  builder::SimpleNetworkBuilder b(/*cell_size=*/ 1);

  // Build intersections.
  auto* i1 = b.build_intersection(/*max_velocity=*/ 2, 50, 50);
  auto* i2 = b.build_intersection(/*max_velocity=*/ 2, 800, 100);
  auto* i3 = b.build_intersection(/*max_velocity=*/ 2, 700, 800);
  auto* i4 = b.build_intersection(/*max_velocity=*/ 2, 20, 400);

  // Build streets.
  auto* s1 = i1->connect_one_way(i2, 5);
  auto* s2 = i2->connect_one_way(i3, 5);
  auto* s3 = i3->connect_one_way(i4, 5);
  auto* s4 = i3->connect_one_way(i1, 5);
  auto* s5 = i4->connect_one_way(i1, 5);
  b.build();

  // Build traffic lights.
  num_traffic_lights = 1;
  lights = new TrafficLight*[num_traffic_lights];
  lights[0] = new TrafficLight(100, {
      new SharedSignalGroup({ s4->last_cell() }),
      new SharedSignalGroup({ s5->last_cell() }) });
  lights[0]->initialize();

  num_cells = b.num_cells();
  cells = new Cell*[num_cells];
  b.get_cells(cells);
}

void init_cars() {
  // Build cars.
  cars = new Car*[num_cars];
  for (int i = 0; i < num_cars; ++i) {
    cars[i] = new Car(5, cells[rand() % num_cells]);
  }
}

void step() {
  for (int i = 0; i < num_traffic_lights; ++i) {
    lights[i]->step();
  }

  for (int i = 0; i < num_cars; ++i) {
    cars[i]->step_velocity();
  }

  for (int i = 0; i < num_cars; ++i) {
    cars[i]->step_move();
  }
}

int main() {
  init_network();
  init_gui(num_cells);
  update_gui();

  init_cars();
  update_gui();

  for (int i = 0; i < num_cells; ++i) {
    cells[i]->draw();
  }
  update_gui();

  while (true) {
    step();
    update_gui();
  }

  destroy_gui();
}