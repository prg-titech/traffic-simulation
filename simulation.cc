#include <chrono>
#include <thread>

#include "drawing.h"
#include "traffic.h"
#include "simple_network_builder.h"

int num_cells;
Cell** cells;

int num_cars = 10;
Car** cars;

void init_network() {
  builder::SimpleNetworkBuilder b(/*cell_size=*/ 5);

  // Build intersections.
  auto* i1 = b.build_intersection(50, 50);
  auto* i2 = b.build_intersection(400, 50);
  auto* i3 = b.build_intersection(350, 400);
  auto* i4 = b.build_intersection(20, 200);

  // Build streets.
  i1->connect_one_way(i2);
  i2->connect_one_way(i3);
  i3->connect_one_way(i4);
  i4->connect_one_way(i1);
  b.build();

  num_cells = b.num_cells();
  cells = new Cell*[num_cells];
  b.get_cells(cells);

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
  init_gui();
  init_network();

  for (int i = 0; i < num_cells; ++i) {
    cells[i]->draw();
  }

  for (int i = 0; i < 100; ++i) {
    step();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}