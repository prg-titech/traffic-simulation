#include <math.h>

#include "simple_network_builder.h"

namespace builder {

void Intersection::connect_one_way(Intersection* other, int max_velocity) {
  builder_->build_street(this, other, max_velocity);
}

void Intersection::connect_two_way(Intersection* other, int max_velocity) {
  this->connect_one_way(other, max_velocity);
  other->connect_one_way(this, max_velocity);
}

// Add an incoming street to this intersection.
void Intersection::connect_incoming(Street* street) {
  incoming_streets_.push_back(street);
}

// Add an outgoing street to this intersection.
void Intersection::connect_outgoing(Street* street) {
  outgoing_streets_.push_back(street);
}

void Intersection::build() {
  // Simple case: Connect every incoming street to every outgoing street.
  for (auto out = outgoing_streets_.begin();
       out != outgoing_streets_.end(); ++out) {
    for (auto in = incoming_streets_.begin();
         in != incoming_streets_.end(); ++in) {
      (*in)->last_cell()->connect_to((*out)->first_cell());
    }
  }
}

Street::Street(SimpleNetworkBuilder* builder, Intersection* from,
               Intersection* to, int max_velocity) {
  // STEP 1: Create street.
  double dx = to->x() - from->x();
  double dy = to->y() - from->y();
  double dist = sqrt(dx*dx + dy*dy);
  int num_segments = dist / builder->cell_size();

  // Size of one segment/cell.
  double segment_dx = dx / num_segments;
  double segment_dy = dy / num_segments;

  // Build first cell.
  Cell* prev_segment = builder->build_cell(from->x() + 0.5*segment_dx,
                                           from->y() + 0.5*segment_dy,
                                           max_velocity);
  first_cell_ = prev_segment;

  // Build remaining cells.
  for (int i = 1; i < num_segments; ++i) {
    Cell* new_segment = builder->build_cell(from->x() + (i+0.5)*segment_dx,
                                            from->y() + (i+0.5)*segment_dy,
                                            max_velocity);
    prev_segment->connect_to(new_segment);
    prev_segment = new_segment;
  }

  last_cell_ = prev_segment;
  target_ = to;

  // STEP 2: Connect street to other streets.
  from->connect_outgoing(this);
  to->connect_incoming(this);
}

}  // namespace builder
