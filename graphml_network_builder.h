#ifndef GRAPHML_NETWORK_BUILDER_H
#define GRAPHML_NETWORK_BUILDER_H

#include <map>
#include <string>
#include <vector>
#include "lib/rapidxml-1.13/rapidxml.hpp"

#include "simple_network_builder.h"
#include "traffic.h"

class Cell;

using namespace std;

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
  void build_traffic_lights();

  double max_x() { return max_x_; }
  double max_y() { return max_y_; }

  Simulation* simulation() { return simulation_; }

 private:
  Simulation* simulation_;

  SimpleNetworkBuilder builder_;

  double max_x_, max_y_;

  // Mapping from XML values to cell types.
  static const map<string, Cell::Type> cell_types_;

  static map<string, Cell::Type> create_cell_types_map();
};

}  // namespace builder

#endif  // GRAPHML_NETWORK_BUILDER_H
