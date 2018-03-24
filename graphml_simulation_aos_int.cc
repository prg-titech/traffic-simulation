namespace simulation {
namespace standard {
class Simulation;
Simulation* instance;
}  // namespace standard

namespace aos_int {
class Simulation;
Simulation* instance;
}  // namespace aos_int
}  // namespace simulation

#include "option_standard.inc"
#include "graphml_simulation.inc"

using namespace std;

#include "traffic_aos_int.h"

int main(int argc, char** argv) {
  simulation::standard::instance = build_simulation(argc, argv);
  simulation::aos_int::instance = new simulation::aos_int::Simulation(
      simulation::standard::instance);

  printf("|      it/s | active | jammed |   turn | checksum \nPerformance: (computing)");
  fflush(stdout);

  last_time = std::chrono::steady_clock::now();

  while (true) {
    step(simulation::aos_int::instance);
    renderer->update_gui();
  }

  delete renderer;

  return 0;
}
