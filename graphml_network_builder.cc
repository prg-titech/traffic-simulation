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

  double t_max_x_ = -std::numeric_limits<double>::max();
  double t_min_x_ = std::numeric_limits<double>::max();
  double t_max_y_ = -std::numeric_limits<double>::max();
  double t_min_y_ = std::numeric_limits<double>::max();
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

GraphmlNetworkBuilder::GraphmlNetworkBuilder(
    string filename, int cell_size, double default_speed_limit,
    double iteration_length) : builder_(cell_size) {
  // Max. velocity is measured in cells / iteration length.
  int max_velocity = default_speed_limit / cell_size * iteration_length;
  if (max_velocity == 0) {
    cout << "Invalid configuration: Resolution is too small.\n";
    exit(1);
  }
  int intersection_speed_limit = max_velocity / 2;

  cout << "Reading graphml file: " << filename << "\n";
  cout << "Iteration length = " << iteration_length << " s\n";
  cout << "Cell size = " << cell_size << " m\n";
  cout << "Default speed = " << default_speed_limit << " m/s\n";
  cout << "              = " << max_velocity << " cells/iteration\n";
  cout << "Intersection speed limit = " << intersection_speed_limit
       << " cells/iteration\n";

  std::ifstream xml_stream(filename.c_str());
  std::string xml_content;

  xml_stream.seekg(0, std::ios::end);   
  xml_content.reserve(xml_stream.tellg());
  xml_stream.seekg(0, std::ios::beg);

  xml_content.assign((std::istreambuf_iterator<char>(xml_stream)),
                      std::istreambuf_iterator<char>());

  xml_document<> doc;
  doc.parse<0>(const_cast<char*>(xml_content.c_str()));

  auto* graph_ml_node = doc.first_node("graphml");
  assert(graph_ml_node != NULL);

  // Parse layout description.
  map<string, string> data_ids;
  data_ids["x"] = data_ids["y"] = data_ids["geometry"]
                = data_ids["length"] = data_ids["osmid"]
                = data_ids["oneway"] = data_ids["maxspeed"] = "n/a";
  for (auto* node = graph_ml_node->first_node("key"); node;
       node = node->next_sibling("key")) {
    if (data_ids.find(node->first_attribute("attr.name")->value())
        != data_ids.end()) {
      data_ids[node->first_attribute("attr.name")->value()] =
          node->first_attribute("id")->value();
      cout << "Graph file has data for "
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
          data_ids["osmid"].c_str()) == 0) {
        id = atol(data_node->value());
      }
      else if (strcmp(data_node->first_attribute("key")->value(),
               data_ids["y"].c_str()) == 0) {
        pos_lat = atof(data_node->value());
        min_lat = min(min_lat, pos_lat);
        max_lat = max(max_lat, pos_lat);
      }
      else if (strcmp(data_node->first_attribute("key")->value(),
               data_ids["x"].c_str()) == 0) {
        pos_long = atof(data_node->value());
        min_long = min(min_long, pos_long);
        max_long = max(max_long, pos_long);
      }
    }

    nodes[id] = make_pair(pos_lat, pos_long);
  }
  cout << nodes.size() << " intersections betweeen (" << min_lat  << ", "
       << min_long << ") and (" << max_lat << ", " << max_long << ").\n";

  // Create intersections.
  map<uint64_t, Intersection*> intersections;
  CoordinateTranslator ctrans(min_long, min_lat, max_long, max_lat);
  for (auto it = nodes.begin(); it != nodes.end(); ++it) {
    intersections[it->first] = builder_.build_intersection(
        intersection_speed_limit, ctrans.x(it->second.second),
        ctrans.y(it->second.first),
        it->second.first, it->second.second);
  }

  // Load and create streets.
  int num_streets = 0;
  for (auto* edge = graph_node->first_node("edge"); edge;
       edge = edge->next_sibling("edge"), ++num_streets) {
    uint64_t from = atol(edge->first_attribute("source")->value());
    assert(intersections.find(from) != intersections.end());
    uint64_t to = atol(edge->first_attribute("target")->value());
    assert(intersections.find(to) != intersections.end());

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
          data_ids["oneway"].c_str()) == 0) {
        is_one_way = strcmp(data_edge->value(), "True") == 0;
      }
      else if (strcmp(data_edge->first_attribute("key")->value(),
               data_ids["length"].c_str()) == 0) {
        length = atof(data_edge->value());
      }
      else if (strcmp(data_edge->first_attribute("key")->value(),
               data_ids["geometry"].c_str()) == 0) {
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
            shape.push_back(builder_.build_intersection(
                intersection_speed_limit, pos_x, pos_y));
          }
        } else {
          cout << "Warning: Expected LINESTRING but found: \n" << linestring
               << "\n";
        }
      }
      else if (strcmp(data_edge->first_attribute("key")->value(),
               data_ids["maxspeed"].c_str()) == 0) {
        string suffix = " mph";
        string limitstring = data_edge->value();

        if (limitstring.size() > suffix.size()
            && limitstring.compare(limitstring.size() - suffix.size(), 
                                   suffix.size(), suffix) == 0) {
          string speed_limit_string = limitstring.substr(
              0, limitstring.size() - suffix.size());
          int limit_mph = atoi(speed_limit_string.c_str());
          int speed_limit = 0.44704*limit_mph*iteration_length/cell_size;

          if (speed_limit == 0) {
            cout << "Warning: Resolution not high enough for speed limit "
                 << speed_limit_string << " mph. Setting to 1 cell/iteration = "
                 << (cell_size * 1.0/iteration_length) << " m/s.";
          }
        } else {
          cout << "Warning: Expected speed limit in mph but found: \n"
               << limitstring << "\n";
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
      int_start->connect_one_way(int_end, max_velocity);

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

  cout << num_streets << " streets.\n";
  cout << "Content within (" << ctrans.min_x() << ", " << ctrans.min_y()
       << ") and (" << ctrans.max_x() << ", " << ctrans.max_y() << ")\n";

  max_x_ = ctrans.max_x();
  max_y_ = ctrans.max_y();
}

void GraphmlNetworkBuilder::build() {
  builder_.build();
}

int GraphmlNetworkBuilder::num_cells() {
  return builder_.num_cells();
}

void GraphmlNetworkBuilder::get_cells(Cell** cells) {
  builder_.get_cells(cells);
}

std::vector<Street*>& GraphmlNetworkBuilder::streets() {
  return builder_.streets();
}

}  // namespace builder
