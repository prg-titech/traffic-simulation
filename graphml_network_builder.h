#ifndef GRAPHML_NETWORK_BUILDER_H
#define GRAPHML_NETWORK_BUILDER_H

#include <map>
#include <string>
#include <vector>
#include "lib/rapidxml-1.13/rapidxml.hpp"

#include "simple_network_builder.h"
#include "traffic.h"

using namespace std;
using namespace simulation::standard;

namespace builder {

class GraphmlNetworkBuilder {
 public:
  // Cell size in meters: Should be length of one vehicle, def.: 5 meters.
  // Default speed limit in m/s, def.: 13.8 m/s = 50 km/h.
  // Iteration length in seconds: Higher length means better resolution, but
  // also higher safety distance to previous vehicle. Def.: 2s.
  // Assuming a speed limit of 50 km/h, this implies a safety distance of 25 m.
  GraphmlNetworkBuilder(string filename, int cell_size = 5,
                        double default_speed_limit = 13.8,
                        double iteration_length = 2);

  void build_connections();
  void build_traffic_controllers();

  double max_x() { return max_x_; }
  double max_y() { return max_y_; }

  Simulation* simulation() { return simulation_; }

 private:
  // Only for debugging purposes.
  int num_traffic_lights_ = 0;
  int num_priority_yield_traffic_controllers_ = 0;

  // These counters are used to give unique IDs to those entities.
  unsigned int signal_group_counter_ = 0;
  unsigned int traffic_light_counter_ = 0;
  unsigned int priority_controller_counter_ = 0;

  Simulation* simulation_;

  SimpleNetworkBuilder builder_;

  double max_x_, max_y_;

  TrafficController* build_traffic_light(Intersection* intersection);
  TrafficController* build_priority_yield_traffic_controller(
      Intersection* intersection);

  static Cell::Type parse_cell_type(string str);
  static int parse_speed_limit(string str);

  // Mapping from XML values to cell types.
  static const map<string, Cell::Type> cell_types_;

  static map<string, Cell::Type> create_cell_types_map();
};

}  // namespace builder

#endif  // GRAPHML_NETWORK_BUILDER_H
