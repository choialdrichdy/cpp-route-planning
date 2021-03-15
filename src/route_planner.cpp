#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node *node : current_node->neighbors) {
        node->parent = current_node;
        node->h_value = this->CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        this->open_list.push_back(node);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), [](RouteModel::Node* node_a, RouteModel::Node* node_b) { 
        return (node_a->g_value + node_a->h_value) > (node_b->g_value + node_b->h_value); 
        });
    RouteModel::Node* node = this->open_list.back();
    this->open_list.pop_back();
    return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent != nullptr || current_node->parent == this->start_node) {
        distance += current_node->distance(*current_node->parent);
        path_found.insert(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }
    
    distance += current_node->g_value;
    path_found.insert(path_found.begin(), *current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->start_node->visited = true;
    this->AddNeighbors(this->start_node);
    while (this->open_list.size() > 0) {
        current_node = this->NextNode();
        if (current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
            break;
        }
        this->AddNeighbors(current_node);
    }
    this->m_Model.path = this->ConstructFinalPath(current_node);

}