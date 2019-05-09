#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
  	// data comes 0-100, so we scale to 0-1.
  	start_x *= 0.01;
  	start_y *= 0.01;
  	end_x *= 0.01;
  	end_y *= 0.01;
  
  	start_node = &m_Model.FindClosestNode(start_x, start_y);
  	end_node = &m_Model.FindClosestNode(end_x, end_y);

}

vector<RouteModel::Node> RoutePlanner::ConstructFinalPath( RouteModel::Node *current_node) {
  
  distance = 0.0f;
  vector<RouteModel::Node> path_found = {};
  RouteModel::Node parent;
  
  while (current_node->parent != nullptr) {
    path_found.push_back(*current_node);
    parent = *(current_node->parent);
    distance += current_node->distance(parent);
    current_node = current_node->parent;
  }
  // add last node
  path_found.push_back(*current_node); 
  // scale distance
  distance *= m_Model.MetricScale();
  
  return path_found;
}

void RoutePlanner::AStarSearch() {

  end_node->parent = start_node;
  m_Model.path = ConstructFinalPath(end_node);
  return;
}
