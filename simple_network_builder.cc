#include <math.h>

#include "simple_network_builder.h"

namespace builder {

Street* Intersection::connect_one_way(Intersection* other,
                                      double max_velocity,
                                      Cell::Type type) {
  return builder_->build_street(this, other, max_velocity, type);
}

TwoWayStreet* Intersection::connect_two_way(Intersection* other,
                                            double max_velocity,
                                            Cell::Type type) {
  auto* first = this->connect_one_way(other, max_velocity, type);
  auto* second = other->connect_one_way(this, max_velocity, type);
  return new TwoWayStreet(first, second);
}

// Add an incoming street to this intersection.
void Intersection::connect_incoming(Street* street) {
  incoming_streets_.push_back(street);
}

// Add an outgoing street to this intersection.
void Intersection::connect_outgoing(Street* street) {
  outgoing_streets_.push_back(street);
}

void Intersection::build_connections() {
  if (outgoing_streets_.size() == 0 && incoming_streets_.size() == 0) {
    printf("Warning: Detected intersection without any edges.\n");
    return;
  }

  if (outgoing_streets_.size() == 0) {
    printf("Warning: Intersection has no outgoing edge! Sending to random street.\n");
    auto street = builder_->streets_[rand() % builder_->streets_.size()];
    outgoing_streets_.push_back(street);
  }

  if (incoming_streets_.size() == 0) {
    printf("Warning: Intersection has no incoming edge.\n");
  }

  // Simple case: Connect every incoming street to every outgoing street.
  for (auto out = outgoing_streets_.begin();
       out != outgoing_streets_.end(); ++out) {
    for (auto in = incoming_streets_.begin();
         in != incoming_streets_.end(); ++in) {
      (*in)->last_cell()->connect_to((*out)->first_cell());
      (*in)->last_cell()->set_max_velocity(max_velocity_);
      (*out)->first_cell()->set_max_velocity(max_velocity_);
    }
  }
}

Street::Street(SimpleNetworkBuilder* builder, Intersection* from,
               Intersection* to, double max_velocity, Cell::Type type) {
  if (from == to) {
    printf("Warning: Detected self-loop during street construction.\n");
  }

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
                                           max_velocity, type);
  first_cell_ = prev_segment;

  // Build remaining cells.
  for (int i = 1; i < num_segments; ++i) {
    Cell* new_segment = builder->build_cell(from->x() + (i+0.5)*segment_dx,
                                            from->y() + (i+0.5)*segment_dy,
                                            max_velocity, type);
    prev_segment->connect_to(new_segment);
    prev_segment = new_segment;
  }

  last_cell_ = prev_segment;
  target_ = to;

  // STEP 2: Connect street to other streets.
  from->connect_outgoing(this);
  to->connect_incoming(this);
}

Cell* SimpleNetworkBuilder::build_cell(double x, double y, double max_velocity,
                                       Cell::Type type) {
  if (x < 0 || y < 0) {
    ++cells_out_of_range_;
  }

  auto* cell = new Cell(max_velocity, x, y, type);
  simulation_->add_cell(cell);
  return cell;
}

Street* SimpleNetworkBuilder::build_street(Intersection* from, Intersection* to,
                                           double max_velocity, Cell::Type type) {
  auto* street = new Street(this, from, to, max_velocity, type);
  streets_.push_back(street);

  // Currently only for rendering purposes...
  simulation_->add_street(new ::Street(street->first_cell(),
                                       street->last_cell(), type));
  return street;
}

}  // namespace builder
