#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <vector>
#include "lib/rapidxml-1.13/rapidxml.hpp"
#include <chrono>

#include "traffic.h"
#include "graphml_network_builder.h"
#include "drawing.h"

using builder::GraphmlNetworkBuilder;

static unsigned int kNumCars = 250000;

Renderer* renderer;

void draw_cell(const Cell& cell) {
  renderer->draw_cell(cell);
}

void print_stats(float iterations_per_second, uint64_t checksum) {
  int num_cars_active = 0;
  int num_cars_jammed = 0;
  int num_cars_turning = 0;
  
  for (int i = 0; i < kNumCars; ++i) {
    if (instance->car(i)->is_active()) {
      num_cars_active++;

      if (instance->car(i)->position()->tag() && Cell::kTurnLane) {
        num_cars_turning++;
      }
    }

    if (instance->car(i)->is_jammed()) {
      num_cars_jammed++;
    }
  }

  printf("| %9.4f | %6d | %6d | %6d | %lu\n",
         iterations_per_second, num_cars_active, num_cars_jammed,
         num_cars_turning, checksum % 1234567);
  fflush(stdout);
}

simulation::standard::Simulation* build_simulation(int argc, char** argv,
                                                   unsigned int num_cars) {
  srand(42);
  kNumCars = num_cars;

  string filename(argv[1]);
  GraphmlNetworkBuilder graph_builder(filename);
  graph_builder.build_connections();
  graph_builder.build_traffic_controllers();
  auto* instance = graph_builder.simulation();

  int window_x = 1600;
  int window_y = 1300;
  double scale_factor = 1.0 * window_x / graph_builder.max_x();
  scale_factor = min(scale_factor, 1.0 * window_y / graph_builder.max_y());
  cout << "Using GUI scale factor " << scale_factor << "\n";

  renderer = new Renderer(instance, window_x, window_y, scale_factor);
  renderer->redraw_everything();
  cout << "First GUI update complete.\n";

  // Build cars.
  for (int i = 0; i < num_cars; ++i) {
    uint32_t state = (uint32_t) rand() + 1;
    instance->add_car(new Car(
        i, 10 + (rand() % 20), instance->random_free_cell(), state));
  }

  // Render initial state only.
  renderer->update_gui();

  // Initialize simulation.
  instance->initialize();
  instance->print_stats();

  return instance;
}

auto last_time = std::chrono::steady_clock::now();
int iteration_counter = 0;

template<typename T>
void step(T* instance) {
  if (iteration_counter % 100 == 0) {
    auto current_time = std::chrono::steady_clock::now();
    double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_time).count() / 1000.0;
    print_stats(100.0/seconds, instance->checksum());
    
    last_time = std::chrono::steady_clock::now();
    iteration_counter = 0;
  }

  instance->step();

  // Measure performance.
  ++iteration_counter;
}