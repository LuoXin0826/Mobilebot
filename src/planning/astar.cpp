#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>

#include <algorithm>
#include <queue>
#include <cmath>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    using namespace std;

    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);

    Point<double> init_global(start.x, start.y);
    Point<double> goal_global(goal.x, goal.y);

    Point<int> init_cell = global_position_to_grid_cell(init_global, distances);
    Point<int> goal_cell = global_position_to_grid_cell(goal_global, distances);

    vector<vector<Node*>> nodeMap;

    nodeMap.resize(distances.widthInCells());
    for (int i = 0; i < distances.widthInCells(); i++) {
        nodeMap[i].resize(distances.heightInCells());
        for (int j = 0; j< distances.heightInCells(); j++) {
            nodeMap[i][j] = new Node(i,j,0.0f);
        }
    }

    auto node = nodeMap[init_cell.x][init_cell.y];
    node -> status = open;
    node -> g_score = 0.0f;
    node -> h_score = diagonal_distance(init_cell,goal_cell);
    node -> f_score = node -> g_score + node -> h_score;

    vector<Node*> OPEN;
    Comp comp;
    OPEN.push_back(node);

    int delta = 1;
    while(!OPEN.empty()) {
        Node* node = OPEN.back();
        OPEN.pop_back();
        Point<int> current_cell(node->x, node->y);
        if(diagonal_distance(current_cell, goal_cell) < delta) {
            reconstruct_path(node, nodeMap, path, distances);
            path.path.push_back(goal);
            path.path_length = path.path.size();  
            return path;
            // if (is_path_safe(path,distances,params)) {
            //     return path;  
            // } else {
            //     robot_path_t path;
            //     path.utime = start.utime;
            //     path.path.push_back(start);
            //     path.path_length = path.path.size();
            //     return path;
            // }
                 
        }
        node -> status = closed;

        for (int i = max(node->x - delta, 0); i < min(node->x + delta + 1, distances.widthInCells()); i += delta) {
            for (int j = max(node->y - delta, 0); j < min(node->y + delta + 1, distances.heightInCells()); j += delta) {
                if (i == node->x && j == node->y) continue;
                Node* neighbor = nodeMap[i][j];
                if (neighbor->status == closed) {
                    continue;
                }
                if (distances(i,j) < params.minDistanceToObstacle) {
                    neighbor->status = closed;
                    continue;
                }
                Point<int> neighbor_cell(neighbor->x, neighbor->y);
                float g_temp = node->g_score + diagonal_distance(current_cell, neighbor_cell);
                if (neighbor->status == unknown) {
                    neighbor->status = open;
                    OPEN.push_back(neighbor);
                }
                else if (g_temp > neighbor->g_score) {
                    continue;
                }
                neighbor->theta = compute_angle(node, neighbor);
                neighbor->parent = node;
                neighbor->g_score = g_temp;
                neighbor->f_score = neighbor->g_score + diagonal_distance(neighbor_cell, goal_cell);
                if (distances(i,j) < params.maxDistanceWithCost) {
                    neighbor->f_score += pow(params.maxDistanceWithCost - distances(i,j), params.distanceCostExponent);
                }
                sort(OPEN.begin(),OPEN.end(),comp);
            }
        }
    }
    path.path_length = path.path.size();
    return path;
}



Node::Node(int x_, int y_, float theta_) {
    x = x_;
    y = y_;
    theta = theta_;
    f_score = 1000000000.0;
    g_score = 1000000000.0;
    h_score = 1000000000.0;
    parent = NULL;
    status = unknown;
}

float diagonal_distance(Point<int> A, Point<int> B) {
    int dx = abs(A.x - B.x);
    int dy = abs(A.y - B.y);
    float dist = dx + dy + (sqrt(2.0) - 2.0) * std::min(dx, dy); 
    // float dist = dx+dy;
    return dist;
}

// bool is_path_safe(const robot_path_t path, const ObstacleDistanceGrid& distances, const SearchParams& params) {

//     for (int i=0;i<path.path_length();++i) {
        
//         Point<double> coord(pose.x, pose.y);
//         Point<int> cell = global_position_to_grid_cell(coord, distances);
//         if (!distances.isCellInGrid(cell.x,cell.y)) {
//             return false;
//         } else if (distances(cell.x,cell.y)<params.minDistanceToObstacle) {
//             return false;
//         }
            
//     }
// }

float compute_angle(Node* init, Node* goal) {
    float dy = goal->y - init->y;
    float dx = goal->x - init->x;
    float angle = wrap_to_pi(atan2(dy,dx));
    return angle;
}

void reconstruct_path(Node* node, std::vector<std::vector<Node*>>& grid, robot_path_t& path, const ObstacleDistanceGrid& distances) {
    while (node != NULL) {
        pose_xyt_t pose;
        pose.utime = 0;
        Point<int> current_cell(node->x, node->y);
        Point<float> pt = grid_position_to_global_position(current_cell, distances);
        pose.x = pt.x;
        pose.y = pt.y;
        pose.theta = wrap_to_pi(node->theta);
        path.path.push_back(pose);
        node = node->parent;
    }
}
