#include <cassert>
#include <queue>
#include <utility>

#include "drawing.h"
#include "traffic.h"
#include "random.h"

using namespace std;

extern void draw_cell(const Cell& cell);

namespace simulation {
namespace standard {

// Singleton simulation instance.
extern Simulation* instance;

// Constructors.
Cell::Cell(IndexType id, int max_velocity, double x, double y,
           Type type, uint32_t tag)
    : id_(id), is_free_(true), max_velocity_(max_velocity), type_(type),
      controller_max_velocity_(max_velocity), x_(x), y_(y), tag_(tag) {
  assert(type >= 0 && type < Type::kMaxType);
  assert(max_velocity_ > 0);
}

TrafficLight::TrafficLight(
    IndexType id, int phase_time,
    std::vector<SharedSignalGroup*> signal_groups)
    : id_(id), phase_time_(phase_time), signal_groups_(signal_groups) {
  // Initialize with random time.
  timer_ = rand() % phase_time_;
}

// Functions for initialization/drawing.
void Cell::connect_to(Cell* other) {
  assert(other != this);
  outgoing_cells_.push_back(other);
  other->incoming_cells_.push_back(this);
}

void Cell::draw() const {
  draw_cell(*this);
}

// Simulation logic.
#include "traffic_logic.inc"

}  // namespace standard
}  // namespace simulation
