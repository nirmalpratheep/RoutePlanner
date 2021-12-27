#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    m_Model = model;
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto* neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value =  current_node->g_value+current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }
}


bool Compare(const RouteModel::Node* a,const RouteModel::Node* b) {
   return (a->g_value+a->h_value)>(b->g_value+b->h_value);
}

void RoutePlanner::NodeSort(std::vector<RouteModel::Node*>* open_list) {
    sort(open_list->begin(),open_list->end(),[](const RouteModel::Node* a,const RouteModel::Node* b)
                        { return a->g_value+a->h_value>b->g_value+b->h_value;});    
}

RouteModel::Node *RoutePlanner::NextNode() {
    NodeSort(&open_list);
    auto currentNode = open_list.back();
    open_list.pop_back();
    return currentNode;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node) {
        path_found.push_back(*current_node);
        if(current_node->parent!=nullptr)
            distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return std::move(path_found);
}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    AddNeighbors(current_node);
    current_node->parent = nullptr;
    current_node->visited = true;
    while(!open_list.empty()) {   
        current_node = NextNode();
        if(current_node==end_node) {
            m_Model.path =  ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }
}