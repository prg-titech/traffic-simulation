#ifndef SIMPLE_NETWORK_BUILDER_H
#define SIMPLE_NETWORK_BUILDER_H

#include <vector>
#include <string.h>

#include "traffic.h"

namespace builder {

class SimpleNetworkBuilder;
class Street;

class TwoWayStreet;

class Intersection {
 public:
  Intersection(SimpleNetworkBuilder* builder,
               double max_velocity, double x, double y,
               double latitide = 0.0, double longitude = 0.0)
      : builder_(builder), max_velocity_(max_velocity), x_(x), y_(y),
        latitude_(latitide), longitude_(longitude) {}

  Street* connect_one_way(Intersection* other, double max_velocity);

  TwoWayStreet* connect_two_way(Intersection* other, double max_velocity);

  void connect_incoming(Street* street);

  void connect_outgoing(Street* street);

  void build();

  // TODO: Build traffic controller (e.g., street light).

  double x() { return x_; }
  double y() { return y_; }

  double longitude() { return longitude_; }
  double latitude() { return latitude_; }

 private:
  // Used only for rendering purposes.
  double x_, y_;
  double longitude_, latitude_;

  SimpleNetworkBuilder* builder_;

  // Outgoing streets.
  std::vector<Street*> outgoing_streets_;

  // Incoming streets.
  std::vector<Street*> incoming_streets_;

  double max_velocity_;
};

class Street {
 public:
  Street(SimpleNetworkBuilder* builder, Intersection* from, Intersection* to,
         double max_velocity);

  Cell* first_cell() { return first_cell_; }
  Cell* last_cell() { return last_cell_; }

 private:
  // First and last street segment.
  Cell* first_cell_;
  Cell* last_cell_;

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
  SimpleNetworkBuilder(int cell_size) : cell_size_(cell_size) {}

  ~SimpleNetworkBuilder() {
    for (auto it = intersections_.begin(); it != intersections_.end(); ++it) {
      delete *it;
    }

    for (auto it = streets_.begin(); it != streets_.end(); ++it) {
      delete *it;
    }
  }

  Intersection* build_intersection(
      double max_velocity, double x, double y,
      double latitude = 0.0, double longitude = 0.0) {
    auto* i = new Intersection(this, max_velocity, x, y, latitude, longitude);
    intersections_.push_back(i);
    return i;
  }

  int cell_size() { return cell_size_; }

  Cell* build_cell(double x, double y, double max_velocity) {
    if (x < 0 || y < 0) {
      printf("Warning: Cell (%f, %f) out of range.\n", x, y);
    }

    auto* cell = new Cell(max_velocity, x, y);
    cells_.push_back(cell);
    return cell;
  }

  Street* build_street(Intersection* from, Intersection* to, double max_velocity) {
    auto* street = new Street(this, from, to, max_velocity);
    streets_.push_back(street);
    return street;
  }

  void build() {
    for (auto it = intersections_.begin();
         it != intersections_.end(); ++it) {
      (*it)->build();
    }
  }

  void get_cells(Cell** cells) {
    memcpy(cells, cells_.data(), sizeof(Cell*) * cells_.size());
  }

  int num_cells() { return cells_.size(); }

  std::vector<Street*>& streets() {
    return streets_;
  }

 private:
  int cell_size_;

  std::vector<Intersection*> intersections_;

  std::vector<Cell*> cells_;

  std::vector<Street*> streets_;

};

}

#endif  // SIMPLE_NETWORK_BUILDER_H
