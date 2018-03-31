#ifndef SIMPLE_NETWORK_BUILDER_H
#define SIMPLE_NETWORK_BUILDER_H

#include <vector>
#include <string.h>

#include "traffic.h"

using namespace simulation::standard;

namespace builder {

void print_stats();

class SimpleNetworkBuilder;
class Street;

class TwoWayStreet;

class Intersection {
 public:
  Intersection(SimpleNetworkBuilder* builder, double x, double y,
               double latitide = 0.0, double longitude = 0.0)
      : builder_(builder), x_(x), y_(y),
        latitude_(latitide), longitude_(longitude) {}

  Street* connect_one_way(Intersection* other, int max_velocity,
                          Cell::Type type = Cell::kResidential);

  TwoWayStreet* connect_two_way(Intersection* other, int max_velocity,
                                Cell::Type type = Cell::kResidential);

  void connect_incoming(Street* street);

  void connect_outgoing(Street* street);

  void build_connections(int turn_lane_length);

  // TODO: Build traffic controller (e.g., street light).

  double x() { return x_; }
  double y() { return y_; }

  double longitude() { return longitude_; }
  double latitude() { return latitude_; }

  std::vector<Street*>& outgoing_streets() {
    return outgoing_streets_;
  }

  std::vector<Street*>& incoming_streets() {
    return incoming_streets_;
  }

 private:
  // Used only for rendering purposes.
  double x_, y_;
  double longitude_, latitude_;

  SimpleNetworkBuilder* builder_;

  // Outgoing streets.
  std::vector<Street*> outgoing_streets_;

  // Incoming streets.
  std::vector<Street*> incoming_streets_;
};

class Street {
 public:
  Street(SimpleNetworkBuilder* builder, Intersection* from, Intersection* to,
         int max_velocity, Cell::Type type = Cell::kResidential);

  Cell* first_cell() { return first_cell_; }
  Cell* last_cell() { return last_cell_; }

  std::vector<Cell*>& last_cells() { return all_last_cells_; }

  void add_turn_lane_last_cell(Cell* cell) {
    all_last_cells_.push_back(cell);
  }

 private:
  // First and last street segment.
  Cell* first_cell_;
  Cell* last_cell_;

  // Includes all "last cells" generated by turn lanes.
  std::vector<Cell*> all_last_cells_;

  Intersection* target_;
};

class TwoWayStreet {
 public:
  TwoWayStreet(Street* first, Street* second)
      : first_(first), second_(second) {}

  Street* first() { return first_; }
  Street* second() { return second_; }

 private:
  Street* first_;
  Street* second_;
};

class SimpleNetworkBuilder {
 public:
  SimpleNetworkBuilder(int cell_size) : cell_size_(cell_size) {
    simulation_ = new Simulation(17);
  }

  ~SimpleNetworkBuilder() {
    for (auto it = intersections_.begin(); it != intersections_.end(); ++it) {
      delete *it;
    }

    for (auto it = streets_.begin(); it != streets_.end(); ++it) {
      delete *it;
    }
  }

  Intersection* build_intersection(
      double x, double y,
      double latitude = 0.0, double longitude = 0.0) {
    auto* i = new Intersection(this, x, y, latitude, longitude);
    intersections_.push_back(i);
    return i;
  }

  void build_connections() {
    for (auto it = intersections_.begin();
         it != intersections_.end(); ++it) {
      (*it)->build_connections(/*turn_lane_length=*/ 5);
    }

    printf("%i cell coordinates were out of range.\n", num_cells_out_of_range_);
    printf("Additional turn lane cells: %d\n", num_turn_lane_cells_);
    printf("Number of streets too short for turn lanes: %d\n",
           num_streets_too_short_);
    printf("Number of streets (incl. segments): %lu\n", streets_.size());
    printf("Number of intersections without incoming streets: %d\n",
           num_no_incoming_edge_);
    printf("Number of intersections without outgoing streets: %d\n",
           num_no_outgoing_edge_);
    printf("Number of intersections without any streets: %d\n",
           num_intersections_without_edges_);
  }

  int cell_size() { return cell_size_; }

  Simulation* simulation() { return simulation_; }

 private:
  friend class GraphmlNetworkBuilder;
  friend class Intersection;
  friend class Street;

  // Debug information. x, y is -inf or +inf.
  int num_cells_out_of_range_ = 0;
  int num_turn_lane_cells_ = 0;
  int num_streets_too_short_ = 0;
  int num_no_incoming_edge_ = 0;
  int num_no_outgoing_edge_ = 0;
  int num_intersections_without_edges_ = 0;

  int cell_size_;
  unsigned int cell_counter_ = 0;

  Simulation* simulation_;

  std::vector<Intersection*> intersections_;

  std::vector<Street*> streets_;

  SimpleNetworkBuilder(Simulation* simulation, int cell_size)
      : cell_size_(cell_size), simulation_(simulation) {}

  Cell* build_cell(double x, double y, int max_velocity,
                   Cell::Type type = Cell::kResidential,
                   uint32_t tag = 0);

  Street* build_street(Intersection* from, Intersection* to,
                       int max_velocity, Cell::Type type = Cell::kResidential);
};

}

#endif  // SIMPLE_NETWORK_BUILDER_H
