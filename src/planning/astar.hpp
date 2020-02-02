#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>

#include <common/point.hpp>
#include <common/angle_functions.hpp>
#include <common/grid_utils.hpp>

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

enum Status {open,closed,unknown};

struct Node{
    Node(int x_, int y_, float theta_);
    int x;
    int y;
    float theta;
    float f_score;
    float h_score;
    float g_score;  
    Node* parent;
    Status status;
};

class Comp{
public:
    bool operator()(Node* pl, Node* pr){
        return (pl->f_score > pr->f_score);
    }
};

float diagonal_distance(Point<int> A, Point<int> B);
// bool is_path_safe(const robot_path_t path, const ObstacleDistanceGrid& distances, const SearchParams& params);
float compute_angle(Node* init,Node* goal);
void construct_path(Node* ptr,std::vector<std::vector<Node*>>& grid, robot_path_t& path, const ObstacleDistanceGrid& distances);

#endif // PLANNING_ASTAR_HPP
