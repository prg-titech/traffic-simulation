#include <chrono>
#include <thread>

#include "drawing.h"
#include "traffic.h"
#include "simple_network_builder.h"

int num_cells;
Cell** cells;

int num_cars = 500;
Car** cars;

void init_network() {
  builder::SimpleNetworkBuilder b(/*cell_size=*/ 1);

  // Build intersections.
  auto* i1 = b.build_intersection(50, 50);
  auto* i2 = b.build_intersection(800, 100);
  auto* i3 = b.build_intersection(700, 800);
  auto* i4 = b.build_intersection(20, 400);

  // Build streets.
  i1->connect_one_way(i2, 5);
  i2->connect_one_way(i3, 5);
  i3->connect_one_way(i4, 5);
  i4->connect_one_way(i1, 0);
  b.build();

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