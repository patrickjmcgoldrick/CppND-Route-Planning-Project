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

float RoutePlanner::CalculateHValue(const RouteModel::Node *current_node) {
  return current_node->distance(*end_node);
}

bool compare(const auto &n1, const auto &n2) {
  // calculate f-values
  float f1 = n1->g_value + n1->h_value;
  float f2 = n2->g_value + n2->h_value;

  return (f1 < f2);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node){

  current_node->FindNeighbors();

  for (RouteModel::Node *neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);

    // Add neighbor to open list
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

RouteModel::Node * RoutePlanner::NextNode() {
  // sort open_list according to f-value = (g-value + h-value)
  sort(open_list.begin(), open_list.end(), [](const auto &n1, const auto &n2){
    return (n1->g_value + n1->h_value) < (n2->g_value + n2->h_value);
  });

  // create copy of pointer to lowest f-value node
  RouteModel::Node *least_f_value = open_list.front();
  // delete node from open list
  open_list.erase(open_list.begin());
  // return pointer to node found
  return least_f_value;
}


void RoutePlanner::AStarSearch() {

  start_node->visited = true;
  open_list.push_back(start_node);
  RouteModel::Node *current_node = nullptr;

  // while we have nodes to explore
  while (open_list.size() > 0) {
    // pick the most promising node
    current_node = NextNode();

    // if we are now at the goal node, we're done.
    if (current_node->distance(*end_node) == 0) {

      // A* complete, we have a path
      m_Model.path = ConstructFinalPath(current_node);
      return;

    }

    // continue A* Search
    AddNeighbors(current_node);
    
  }
}
