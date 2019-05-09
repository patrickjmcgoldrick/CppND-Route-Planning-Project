#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"
using std::vector;

class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
	 
    float GetDistance() const { return distance; }
    void AStarSearch();
  
  private:
    // Add private variables or methods declarations here.
	  
    vector<RouteModel::Node> ConstructFinalPath( RouteModel::Node *current_node);
    float CalculateHValue(const RouteModel::Node * node);
    void AddNeighbors(RouteModel::Node * node);
    RouteModel::Node * NextNode();

    RouteModel &m_Model;
  	RouteModel::Node *start_node;
  	RouteModel::Node *end_node;
    vector<RouteModel::Node *> open_list;
  	float distance;
};
