#include "route_model.h"
#include <iostream>
#include <vector>
using std::vector;

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // create a RouteModel:Node for each Model::Node
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
      m_Nodes.push_back(Node(counter, this, node));
      counter++;
    }

  	CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
  for (const Model::Road &road: Roads()) {
    
    // check not a footway
    if (road.type != Model::Road::Type::Footway) {
      
      for (int node_idx : Ways()[road.way].nodes) {
        
		// if not in list already
        if (node_to_road.find(node_idx) == node_to_road.end()) {
          node_to_road[node_idx] = vector<const Model::Road *> {};
        }
        // add road to idx's list of roads
       	node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
  Node node = Node();
  node.x = x;
  node.y = y;
  
  float min_dist = std::numeric_limits<float>::max();
  int closest_idx;
  float dist;
  
  for (const Model::Road &road: Roads()) {
  	if (road.type != Model::Road::Type::Footway) {
      for (int node_idx: Ways()[road.way].nodes) {
        dist = node.distance(SNodes()[node_idx]);
        if (dist < min_dist) {
          closest_idx = node_idx;
          min_dist = dist;
        }
      }
    }
  }
  return this->SNodes()[closest_idx];
}


RouteModel::Node* RouteModel::Node::FindNeighbor(vector<int> node_indices) {
  
  Node *closest_node = nullptr;
  Node node;
  
  for(int node_index: node_indices) {
  	node = parent_model->SNodes()[node_index];
    if (this->distance(node) != 0 && !node.visited) {
      if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
        closest_node = &parent_model->SNodes()[node_index];
      }
    }
  }
  return closest_node;
}

void RouteModel::Node::FindNeighbors() {
  for (auto & road: parent_model->node_to_road[this->index]) {
    RouteModel::Node* new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (new_neighbor) {
      this->neighbors.emplace_back(new_neighbor);
    }
  }
}