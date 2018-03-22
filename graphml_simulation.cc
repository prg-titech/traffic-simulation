#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <chrono>
#include <vector>
#include "lib/rapidxml-1.13/rapidxml.hpp"

namespace simulation {
namespace standard {
class Simulation;
Simulation* instance;
}  // namespace standard
}  // namespace simulation

#include "option_standard.inc"
#include "graphml_simulation.inc"

using namespace simulation::standard;
using namespace std;

int main(int argc, char** argv) {
  instance = build_simulation(argc, argv);

  uint64_t iteration_counter = 0;
  auto last_time = std::chrono::steady_clock::now();

  printf("|      it/s | active | jammed |   turn | checksum \nPerformance: (computing)");
  fflush(stdout);

  while (true) {
    instance->step();
    renderer->update_gui();

    // Measure performance.
    ++iteration_counter;

    if (iteration_counter == 100) {
      auto current_time = std::chrono::steady_clock::now();
      double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(
          current_time - last_time).count() / 1000.0;
      print_stats(100.0/seconds, instance->checksum());
      
      last_time = std::chrono::steady_clock::now();
      iteration_counter = 0;
    }
  }

  delete renderer;

  return 0;
}
