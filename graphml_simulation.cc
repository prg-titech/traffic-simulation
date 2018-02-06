#include <iostream>
#include <vector>
#include "lib/rapidxml-1.13/rapidxml.hpp"

#include "graphml_network_builder.h"
#include "drawing.h"

// TODO: This should all be in namespace builder.
using namespace builder;
using namespace std;

Renderer* renderer;

int num_cars = 40000;
Car** cars;

void draw_cell(Cell* cell) {
  renderer->draw_cell(cell);
}

int main(int argc, char** argv) {
  string filename(argv[1]);
  GraphmlNetworkBuilder graph_builder(filename);
  graph_builder.build_connections();
  graph_builder.build_traffic_lights();
  auto* simulation = graph_builder.simulation();

  cout << simulation->num_cells() << " cells.\n";

  int window_x = 1600;
  int window_y = 1300;
  double scale_factor = 1.0 * window_x / graph_builder.max_x();
  scale_factor = min(scale_factor, 1.0 * window_y / graph_builder.max_y());
  cout << "Using GUI scale factor " << scale_factor << "\n";

  renderer = new Renderer(simulation, window_x, window_y, scale_factor);
  renderer->redraw_everything();
  cout << "First GUI update complete.\n";

  // Build cars.
  for (int i = 0; i < 40000; ++i) {
    simulation->add_car(new Car(20, simulation->random_cell()));
  }
  renderer->update_gui();

  while (true) {
    simulation->step();
    renderer->update_gui();
  }

  delete renderer;

  return 0;
}
