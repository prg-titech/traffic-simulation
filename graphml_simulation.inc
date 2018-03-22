#include "traffic.h"
#include "graphml_network_builder.h"
#include "drawing.h"

using builder::GraphmlNetworkBuilder;

const unsigned int num_cars = 20000;

Renderer* renderer;

void draw_cell(const Cell& cell) {
  renderer->draw_cell(cell);
}

void print_stats(float iterations_per_second, uint64_t checksum) {
  int num_cars_active = 0;
  int num_cars_jammed = 0;
  int num_cars_turning = 0;
  
  for (int i = 0; i < num_cars; ++i) {
    if (instance->cars()[i]->is_active()) {
      num_cars_active++;

      if (instance->cars()[i]->position()->tag() && Cell::kTurnLane) {
        num_cars_turning++;
      }
    }

    if (instance->cars()[i]->is_jammed()) {
      num_cars_jammed++;
    }
  }

  printf("\r| %9.4f | %6d | %6d | %6d | %lu",
         iterations_per_second, num_cars_active, num_cars_jammed,
         num_cars_turning, checksum % 1234567);
  fflush(stdout);
}

simulation::standard::Simulation* build_simulation(int argc, char** argv) {
  srand(42);

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
    uint32_t state2 = (uint32_t) rand() + 1;
    instance->add_car(new Car(
        i, 20, instance->random_free_cell(&state2), state));
  }

  // Render initial state only.
  renderer->update_gui();

  // Initialize simulation.
  instance->initialize();
  instance->print_stats();

  return instance;
}