#ifndef SIMPLE_NETWORK_BUILDER_H
#define SIMPLE_NETWORK_BUILDER_H

#include <vector>
#include <string.h>

#include "traffic.h"

namespace builder {

class SimpleNetworkBuilder;
class Street;

class Intersection {
 public:
  Intersection(SimpleNetworkBuilder* builder, int x, int y)
      : builder_(builder), x_(x), y_(y) {
    assert(x >= 0);
    assert(y >= 0);
  }

  ~Intersection();

  void connect_one_way(Intersection* other);

  void connect_two_way(Intersection* other);

  void connect_incoming(Street* street);

  void connect_outgoing(Street* street);

  void build();

  // TODO: Build traffic controller (e.g., street light).

  int x() { return x_; }
  int y() { return y_; }

 private:
  // Used only for rendering purposes.
  int x_, y_;

  SimpleNetworkBuilder* builder_;

  // Outgoing streets.
  std::vector<Street*> outgoing_streets_;

  // Incoming streets.
  std::vector<Street*> incoming_streets_;
};

class Street {
 public:
  Street(SimpleNetworkBuilder* builder, Intersection* from, Intersection* to);

  Cell* first_cell() { return first_cell_; }
  Cell* last_cell() { return last_cell_; }

 private:
  // First and last street segment.
  Cell* first_cell_;
  Cell* last_cell_;

  Intersection* target_;
};

class SimpleNetworkBuilder {
 public:
  SimpleNetworkBuilder(int cell_size) : cell_size_(cell_size) {}

  ~SimpleNetworkBuilder() {
    for (auto it = intersections_.begin(); it != intersections_.end(); ++it) {
      delete *it;
    }
  }

  Intersection* build_intersection(int x, int y) {
    auto* i = new Intersection(this, x, y);
    intersections_.push_back(i);
    return i;
  }

  int cell_size() { return cell_size_; }

  Cell* build_cell(int x, int y) {
    assert(x >= 0);
    assert(y >= 0);

    auto* cell = new Cell(/*max_velocity=*/ 5, x, y);
    cells_.push_back(cell);
    return cell;
  }

  void build_street(Intersection* from, Intersection* to) {
    new Street(this, from, to);
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

 private:
  int cell_size_;

  std::vector<Intersection*> intersections_;

  std::vector<Cell*> cells_;
};

}

#endif  // SIMPLE_NETWORK_BUILDER_H
