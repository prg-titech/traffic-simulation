#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <chrono>
#include <vector>
#include "lib/rapidxml-1.13/rapidxml.hpp"

#include "graphml_network_builder.h"
#include "drawing.h"

// TODO: This should all be in namespace builder.
using namespace builder;
using namespace std;

Simulation* simulation;
Renderer* renderer;

int num_cars = 20000;
Car* const* cars;

void draw_cell(const Cell& cell) {
  renderer->draw_cell(cell);
}

void print_stats(float iterations_per_second, uint64_t checksum) {
  int num_cars_active = 0;
  int num_cars_jammed = 0;
  int num_cars_turning = 0;
  
  for (int i = 0; i < num_cars; ++i) {
    if (cars[i]->is_active()) {
      num_cars_active++;

      if (cars[i]->position()->tag() && Cell::kTurnLane) {
        num_cars_turning++;
      }
    }

    if (cars[i]->is_jammed()) {
      num_cars_jammed++;
    }
  }

  printf("\r| %9.4f | %6d | %6d | %6d | %lu",
         iterations_per_second, num_cars_active, num_cars_jammed,
         num_cars_turning, checksum);
  fflush(stdout);
}

int main(int argc, char** argv) {
  srand(42);

  string filename(argv[1]);
  GraphmlNetworkBuilder graph_builder(filename);
  graph_builder.build_connections();
  graph_builder.build_traffic_controllers();
  simulation = graph_builder.simulation();

  int window_x = 1600;
  int window_y = 1300;
  double scale_factor = 1.0 * window_x / graph_builder.max_x();
  scale_factor = min(scale_factor, 1.0 * window_y / graph_builder.max_y());
  cout << "Using GUI scale factor " << scale_factor << "\n";

  renderer = new Renderer(simulation, window_x, window_y, scale_factor);
  renderer->redraw_everything();
  cout << "First GUI update complete.\n";

  // Build cars.
  for (int i = 0; i < num_cars; ++i) {
    uint32_t state = (uint32_t) rand() + 1;
    uint32_t state2 = (uint32_t) rand() + 1;
    simulation->add_car(new Car(20, simulation->random_free_cell(&state2),
                                state));
  }
  renderer->update_gui();

  // Initialize simulation.
  simulation->initialize();
  simulation->print_stats();
  cars = simulation->cars().data();

  uint64_t iteration_counter = 0;
  auto last_time = std::chrono::steady_clock::now();

  printf("|      it/s | active | jammed |   turn | checksum \nPerformance: (computing)");
  fflush(stdout);

  while (true) {
    simulation->step();
    renderer->update_gui();

    // Measure performance.
    ++iteration_counter;

    if (iteration_counter == 100) {
      auto current_time = std::chrono::steady_clock::now();
      double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(
          current_time - last_time).count() / 1000.0;
      print_stats(100.0/seconds, simulation->checksum());
      
      last_time = std::chrono::steady_clock::now();
      iteration_counter = 0;
    }
  }

  delete renderer;

  return 0;
}
