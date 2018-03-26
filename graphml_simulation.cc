namespace simulation {
namespace standard {
class Simulation;
Simulation* instance;
}  // namespace standard
}  // namespace simulation

#include "option_standard.inc"
#include "graphml_simulation.inc"

using namespace std;

int main(int argc, char** argv) {
  instance = build_simulation(argc, argv);

  printf("|      it/s | active | jammed |   turn | checksum \nPerformance: (computing)\n");
  fflush(stdout);

  last_time = std::chrono::steady_clock::now();

  while (true) {
    step(instance);
    renderer->update_gui();
  }

  delete renderer;

  return 0;
}
