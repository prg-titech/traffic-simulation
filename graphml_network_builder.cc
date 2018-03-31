#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>
#include <stdlib.h>
#include <map>
#include <math.h>
#include <limits>

#include "graphml_network_builder.h"

using namespace std;
using namespace rapidxml;

namespace builder {

// See also: https://en.wikipedia.org/wiki/Geographic_coordinate_system
class CoordinateTranslator {
 public:
  CoordinateTranslator(double s_min_long, double s_min_lat,
                       double s_max_long, double s_max_lat)
      : s_min_long_(s_min_long), s_min_lat_(s_min_lat),
        s_max_long_(s_max_long), s_max_lat_(s_max_lat) {
    s_avg_lat_ = (s_max_lat_ + s_min_lat_) / 2.0;
  }

  double x(double source_long) {
    double val = long_to_x(source_long) - long_to_x(s_min_long_);
    t_max_x_ = max(val, t_max_x_);
    t_min_x_ = min(val, t_min_x_);
    return val;
  }

  double y(double source_lat) {
    double val = lat_to_y(source_lat) - lat_to_y(s_min_lat_);
    t_max_y_ = max(t_max_y_, val);
    t_min_y_ = min(t_min_y_, val);
    return val;
  }

  double max_x() { return t_max_x_; }
  double min_x() { return t_min_x_; }
  double max_y() { return t_max_y_; }
  double min_y() { return t_min_y_; }

 private:
  double long_to_x(double l) {
    return earth_radius_*(l/180*M_PI)*cos(s_avg_lat_/180*M_PI);
  }

  double lat_to_y(double l) {
    return earth_radius_*l/180*M_PI;
  }

  double earth_radius_ = 6371 * 1000;
  double s_min_long_, s_min_lat_, s_max_long_, s_max_lat_;
  double s_avg_lat_;

  double t_max_x_ = -numeric_limits<double>::max();
  double t_min_x_ = numeric_limits<double>::max();
  double t_max_y_ = -numeric_limits<double>::max();
  double t_min_y_ = numeric_limits<double>::max();
};

// Taken from:
// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
vector<string> split(const string& str, const string& delim)
{
  vector<string> tokens;
  size_t prev = 0, pos = 0;
  do
  {
    pos = str.find(delim, prev);
    if (pos == string::npos) pos = str.length();
    string token = str.substr(prev, pos-prev);
    if (!token.empty()) tokens.push_back(token);
    prev = pos + delim.length();
  }
  while (pos < str.length() && prev < str.length());
  return tokens;
}

map<string, Cell::Type> GraphmlNetworkBuilder::create_cell_types_map() {
  map<string, Cell::Type> result;
  result["service"] = Cell::kService;
  result["residential"] = Cell::kResidential;
  result["unclassified"] = Cell::kUnclassified;
  result["tertiary"] = Cell::kTertiary;
  result["tertiary_link"] = Cell::kTertiary;
  result["secondary"] = Cell::kSecondary;
  result["secondary_link"] = Cell::kSecondary;
  result["primary"] = Cell::kPrimary;
  result["primary_link"] = Cell::kPrimary;
  result["trunk"] = Cell::kTrunk;
  result["trunk_link"] = Cell::kTrunk;
  result["motorway"] = Cell::kMotorway;
  result["motorway_link"] = Cell::kMotorway;
  return result;
}

const map<string, Cell::Type> GraphmlNetworkBuilder::cell_types_ =
    GraphmlNetworkBuilder::create_cell_types_map();

GraphmlNetworkBuilder::GraphmlNetworkBuilder(string filename,
                                             int cell_size,
                                             double default_speed_limit,
                                             double iteration_length)
    : simulation_(new Simulation(17)), builder_(simulation_, cell_size) {
  // Max. velocity is measured in cells / iteration length.
  int max_velocity = default_speed_limit / cell_size * iteration_length;
  if (max_velocity == 0) {
    cout << "Invalid configuration: Resolution is too small.\n";
    exit(1);
  }

  cout << "Reading graphml file: " << filename << "\n";
  cout << "Iteration length = " << iteration_length << " s\n";
  cout << "Cell size = " << cell_size << " m\n";
  cout << "Default speed = " << default_speed_limit << " m/s\n";
  cout << "              = " << max_velocity << " cells/iteration\n";

  ifstream xml_stream(filename.c_str());
  string xml_content;

  xml_stream.seekg(0, ios::end);   
  xml_content.reserve(xml_stream.tellg());
  xml_stream.seekg(0, ios::beg);

  xml_content.assign((istreambuf_iterator<char>(xml_stream)),
                      istreambuf_iterator<char>());

  xml_document<> doc;
  doc.parse<0>(const_cast<char*>(xml_content.c_str()));

  auto* graph_ml_node = doc.first_node("graphml");
  assert(graph_ml_node != NULL);

  // Parse layout description.
  map<string, string> edge_data_ids;
  map<string, string> node_data_ids;
  node_data_ids["x"] = node_data_ids["y"] = node_data_ids["osmid"] = "n/a";
  edge_data_ids["oneway"] = edge_data_ids["length"] = edge_data_ids["geometry"]
                          = edge_data_ids["maxspeed"]= edge_data_ids["highway"]
                          = "n/a";
  for (auto* node = graph_ml_node->first_node("key"); node;
       node = node->next_sibling("key")) {
    if (edge_data_ids.find(node->first_attribute("attr.name")->value())
        != edge_data_ids.end()
        && strcmp(node->first_attribute("for")->value(), "edge") == 0) {
      edge_data_ids[node->first_attribute("attr.name")->value()] =
          node->first_attribute("id")->value();
      cout << "Graph file has edge data for "
           << node->first_attribute("attr.name")->value() << "\n";
    }
    if (node_data_ids.find(node->first_attribute("attr.name")->value())
        != node_data_ids.end()
        && strcmp(node->first_attribute("for")->value(), "node") == 0) {
      node_data_ids[node->first_attribute("attr.name")->value()] =
          node->first_attribute("id")->value();
      cout << "Graph file has node data for "
           << node->first_attribute("attr.name")->value() << "\n";
    }
  }

  auto* graph_node = graph_ml_node->first_node("graph");
  assert(graph_node != NULL);

  // Load intersections.
  double min_long = 200, max_long = -200, min_lat = 100, max_lat = -100;
  map<uint64_t, pair<double, double>> nodes;
  for (auto* node = graph_node->first_node("node"); node;
       node = node->next_sibling("node")) {
    uint64_t id;
    double pos_long, pos_lat;

    for (auto* data_node = node->first_node("data"); data_node;
         data_node = data_node->next_sibling("data")) {

      if (strcmp(data_node->first_attribute("key")->value(),
          node_data_ids["osmid"].c_str()) == 0) {
        id = atol(data_node->value());
      }
      else if (strcmp(data_node->first_attribute("key")->value(),
               node_data_ids["y"].c_str()) == 0) {
        pos_lat = atof(data_node->value());
        min_lat = min(min_lat, pos_lat);
        max_lat = max(max_lat, pos_lat);
      }
      else if (strcmp(data_node->first_attribute("key")->value(),
               node_data_ids["x"].c_str()) == 0) {
        pos_long = atof(data_node->value());
        min_long = min(min_long, pos_long);
        max_long = max(max_long, pos_long);
      }
    }

    nodes[id] = make_pair(pos_lat, pos_long);
  }

  // Create intersections.
  map<uint64_t, Intersection*> intersections;
  CoordinateTranslator ctrans(min_long, min_lat, max_long, max_lat);
  for (auto it = nodes.begin(); it != nodes.end(); ++it) {
    intersections[it->first] = builder_.build_intersection(
        ctrans.x(it->second.second),
        ctrans.y(it->second.first),
        it->second.first, it->second.second);
  }

  cout << nodes.size() << " intersections betweeen (" << min_lat  << ", "
       << min_long << ") and (" << max_lat << ", " << max_long << ").\n";
  cout << "In translated coordiates: (" << ctrans.min_x() << ", "
       << ctrans.min_y() << ") and (" << ctrans.max_x() << ", "
       << ctrans.max_y() << ")\n";

  // Load and create streets.
  int num_streets = 0;
  for (auto* edge = graph_node->first_node("edge"); edge;
       edge = edge->next_sibling("edge"), ++num_streets) {
    uint64_t from = atol(edge->first_attribute("source")->value());
    assert(intersections.find(from) != intersections.end());
    uint64_t to = atol(edge->first_attribute("target")->value());
    assert(intersections.find(to) != intersections.end());

    Cell::Type street_type = Cell::kResidential;
    bool is_one_way = false;
    double length;
    int speed_limit = max_velocity;

    // A list of (possibly two-way) intersections that defines the shape
    // of this street.
    vector<Intersection*> shape;
    shape.push_back(intersections[from]);

    for (auto* data_edge = edge->first_node("data"); data_edge;
         data_edge = data_edge->next_sibling("data")) {
      if (strcmp(data_edge->first_attribute("key")->value(),
          edge_data_ids["oneway"].c_str()) == 0) {
        is_one_way = strcmp(data_edge->value(), "True") == 0;
      }
      else if (strcmp(data_edge->first_attribute("key")->value(),
               edge_data_ids["length"].c_str()) == 0) {
        length = atof(data_edge->value());
      }
      else if (strcmp(data_edge->first_attribute("key")->value(),
               edge_data_ids["geometry"].c_str()) == 0) {
        // Parse shape description.
        string prefix = "LINESTRING (";
        string linestring = data_edge->value();
        if (linestring.compare(0, prefix.size(), prefix) == 0) {
          string coordinate_string = linestring.substr(
              prefix.size(), linestring.size() - prefix.size() - 1);
          auto coordinates = split(coordinate_string, ", ");

          for (int i = 0; i < coordinates.size(); ++i) {
            // Split by latitude, longitude.
            auto long_lat = split(coordinates[i], " ");
            double pos_x = ctrans.x(atof(long_lat[0].c_str()));
            double pos_y = ctrans.y(atof(long_lat[1].c_str()));
            shape.push_back(builder_.build_intersection(pos_x, pos_y));
          }
        } else {
          cout << "Warning: Expected LINESTRING but found: \n" << linestring
               << "\n";
        }
      }
      else if (strcmp(data_edge->first_attribute("key")->value(),
               edge_data_ids["maxspeed"].c_str()) == 0) {
        string suffix = " mph";
        string limitstring = data_edge->value();

        if (limitstring.size() > suffix.size()
            && limitstring.compare(limitstring.size() - suffix.size(), 
                                   suffix.size(), suffix) == 0) {
          string speed_limit_string = limitstring.substr(
              0, limitstring.size() - suffix.size());
          int limit_mph = atoi(speed_limit_string.c_str());
          speed_limit = 0.44704*limit_mph*iteration_length/cell_size;

          if (speed_limit == 0) {
            cout << "Warning: Resolution not high enough for speed limit "
                 << speed_limit_string << " mph. Setting to 1 cell/iteration = "
                 << (cell_size * 1.0/iteration_length) << " m/s.";
            speed_limit = 1;
          }
        } else {
          int limit_mph = atoi(limitstring.c_str());
          if (limit_mph > 0 && limit_mph <= 200) {
            // Seems like valid speed limit.
            speed_limit = 0.44704*limit_mph*iteration_length/cell_size;
            if (speed_limit == 0) {
              speed_limit = 1;
            }
          } else {
            cout << "Warning: Expected speed limit in mph but found: "
                 << limitstring << "\n";
          }
        }
      }
      else if (strcmp(data_edge->first_attribute("key")->value(),
               edge_data_ids["highway"].c_str()) == 0) {
        if (cell_types_.find(data_edge->value()) != cell_types_.end()) {
          street_type = cell_types_.at(data_edge->value());
        } else {
          cout << "Warning: Unknown cell type: "
               << data_edge->value() << "\n";
        }
      }
    }

    shape.push_back(intersections[to]);

    // Check if shape matches length.
    double calculated_len = 0.0;
    // Generate multiple streets: One for every pair of shape intersections.
    for (int i = 0; i < shape.size() - 1; ++i) {
      auto* int_start = shape[i];
      auto* int_end = shape[i + 1];

      // TODO: Need to adjust layout if two-way intersection.
      assert(int_start != NULL);
      assert(int_end != NULL);
      assert(street_type >= 0 && street_type < Cell::kMaxType);
      int_start->connect_one_way(int_end, speed_limit, street_type);

      // Check if shape matches length.
      double dx = int_end->x() - int_start->x();
      double dy = int_end->y() - int_start->y();
      calculated_len += sqrt(dx*dx + dy*dy);
    }

    // Print warning if length differs by more than 2 meters.
    if (fabs(calculated_len - length) > 2) {
      cout << "Warning: Stored length of street " << length << " differs from "
           << "calulated one " << calculated_len << "\n";
    }
  }

  builder::print_stats();
  cout << num_streets << " streets.\n";
  cout << "Content within (" << ctrans.min_x() << ", " << ctrans.min_y()
       << ") and (" << ctrans.max_x() << ", " << ctrans.max_y() << ")\n";

  max_x_ = ctrans.max_x();
  max_y_ = ctrans.max_y();
}

void GraphmlNetworkBuilder::build_connections() {
  builder_.build_connections();
}


Cell::Type cell_type(Street* street) {
  return street->last_cell()->type();
}

#ifndef NDEBUG
// Ensure that no cell is used twice in a traffic light.
set<Cell*> traffic_light_cells_;
#endif

TrafficController* GraphmlNetworkBuilder::build_traffic_light(
    Intersection* intersection) {
  auto& incoming = intersection->incoming_streets();
  vector<SharedSignalGroup*> signal_groups;
  for (int j = 0; j < incoming.size(); ++j) {
    signal_groups.push_back(
        new SharedSignalGroup(signal_group_counter_++,
                              incoming[j]->last_cells()));

#ifndef NDEBUG
    for (int i = 0; i < incoming[j]->last_cells().size(); ++i) {
      Cell* next_cell = incoming[j]->last_cells()[i];
      assert(traffic_light_cells_.insert(next_cell).second);
    }
#endif
  }

  int phase_len = 40 + rand() % 20;
  simulation_->add_traffic_controller(new TrafficLight(
      traffic_light_counter_++, phase_len, signal_groups));
  ++num_traffic_lights_;
}

TrafficController*
    GraphmlNetworkBuilder::build_priority_yield_traffic_controller(
        Intersection* intersection) {
  auto& incoming = intersection->incoming_streets();
  auto& outgoing = intersection->outgoing_streets();

  // TODO: Support more than two incoming streets.
  assert(incoming.size() == 2);

  auto* in_0_cells = new SharedSignalGroup(signal_group_counter_++,
                                           incoming[0]->last_cells());
  auto* in_1_cells = new SharedSignalGroup(signal_group_counter_++,
                                           incoming[1]->last_cells());

  if (cell_type(incoming[0]) > cell_type(incoming[1])) {
    simulation_->add_traffic_controller(new PriorityYieldTrafficController(
        priority_controller_counter_++, { in_0_cells, in_1_cells } ));
  } else {
    simulation_->add_traffic_controller(new PriorityYieldTrafficController(
        priority_controller_counter_++, { in_1_cells, in_0_cells } ));
  }
  ++num_priority_yield_traffic_controllers_;
}

void GraphmlNetworkBuilder::build_traffic_controllers() {
  for (int i = 0; i < builder_.intersections_.size(); ++i) {
    auto* intersection = builder_.intersections_[i];
    auto& incoming = intersection->incoming_streets();
    int num_incoming = incoming.size();

    if (num_incoming == 2 && cell_type(incoming[0]) != cell_type(incoming[1])) {
      build_priority_yield_traffic_controller(intersection);
    } else if (num_incoming >= 2) {
      build_traffic_light(intersection);
    }
  }

  cout << "Number of traffic lights: " << num_traffic_lights_ << "\n"
       << "Number of priority yield traffic controllers: "
       << num_priority_yield_traffic_controllers_ << "\n";
}

}  // namespace builder
