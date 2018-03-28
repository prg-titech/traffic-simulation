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

  printf("|      it/s | active | jammed |   turn | checksum \nPerformance: (computing)\n");
  fflush(stdout);

  auto t1 = std::chrono::steady_clock::now();

  for (int i = 0; i < 1000; ++i) {
    instance->step();
    // renderer->update_gui();
  }
  auto t2 = std::chrono::steady_clock::now();
  unsigned long millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      t2 - t1).count();
  auto checksum = instance->checksum();

  printf("Checksum: %lu, CPU Time (millis): %lu\n", checksum, millis);

  delete renderer;

  return 0;
}
