#include <iostream>
#include <vector>
#include "lib/rapidxml-1.13/rapidxml.hpp"

#include "graphml_network_builder.h"
#include "drawing.h"

// TODO: This should all be in namespace builder.
using namespace builder;
using namespace std;

Renderer* renderer;

int num_cells;
Cell** cells;

int num_cars = 40000;
Car** cars;

vector<TrafficLight*> traffic_lights;

void draw_cell(Cell* cell) {
  renderer->draw_cell(cell);
}

void init_cars() {
  // Build cars.
  cars = new Car*[num_cars];
  for (int i = 0; i < num_cars; ++i) {
    cars[i] = new Car(20, cells[rand() % num_cells]);
  }
}

void step() {
  for (int i = 0; i < traffic_lights.size(); ++i) {
    traffic_lights[i]->step();
  }

  for (int i = 0; i < num_cars; ++i) {
    cars[i]->step_velocity();
  }

  for (int i = 0; i < num_cars; ++i) {
    cars[i]->step_move();
  }
}

int main(int argc, char** argv) {
  string filename(argv[1]);
  GraphmlNetworkBuilder graph_builder(filename);
  graph_builder.build();
  num_cells = graph_builder.num_cells();
  cout << num_cells << " cells.\n";
  cells = new Cell*[num_cells];
  graph_builder.get_cells(cells);

  int window_x = 1600;
  int window_y = 1300;
  double scale_factor = 1.0 * window_x / graph_builder.max_x();
  scale_factor = min(scale_factor, 1.0 * window_y / graph_builder.max_y());
  cout << "Using GUI scale factor " << scale_factor << "\n";

  renderer = new Renderer(num_cells, cells, window_x, window_y, scale_factor);

  // Add streets.
  auto& streets = graph_builder.streets();
  for (int i = 0; i < streets.size(); ++i) {
    renderer->add_street(make_tuple(
        streets[i]->first_cell()->x(),
        streets[i]->first_cell()->y(),
        streets[i]->last_cell()->x(),
        streets[i]->last_cell()->y()));
  }
  
  // Add traffic lights.
  traffic_lights = graph_builder.build_traffic_lights();

  renderer->redraw_everything();
  cout << "First GUI update complete.\n";

  init_cars();
  renderer->update_gui();

  while (true) {
    step();
    renderer->update_gui();
  }

  delete renderer;

  return 0;
}
