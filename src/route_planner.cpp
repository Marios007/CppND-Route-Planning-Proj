#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

   
    RoutePlanner::start_node = &model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &model.FindClosestNode(end_x, end_y); 
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(int i=0; i < current_node->neighbors.size(); i++)
    {
        current_node->neighbors[i]->parent = current_node;
        current_node->neighbors[i]->h_value = CalculateHValue(current_node->neighbors[i]);
        current_node->neighbors[i]->g_value =  current_node->g_value + current_node->distance(*current_node->neighbors[i]);
        open_list.push_back(current_node->neighbors[i]);
        current_node->neighbors[i]->visited = true;
    }
}


bool Compare(const RouteModel::Node* a,const RouteModel::Node* b)
{
    float sum_a = a->h_value + a->g_value ;
    float sum_b = b->h_value + b->g_value ;
    return sum_a > sum_b;
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *pt_lowest = open_list.back();
    open_list.pop_back();
    return pt_lowest;   
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    std::vector<RouteModel::Node> path_found;

    while (!(current_node->parent == nullptr))
    {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    start_node->visited= true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;

    while ((open_list.size() > 0))
    {
        current_node =  NextNode();
        if (current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else
        {
        AddNeighbors(current_node);
        }
    }
}