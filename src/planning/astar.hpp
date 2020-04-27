#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/grid_utils.hpp>
#include <common/point.hpp>
#include <queue>

class ObstacleDistanceGrid;
typedef Point<int> cell_t;
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



// class Astar{
//     public:
        
        
//         bool is_node_free(Node n, const ObstacleDistanceGrid& distances);
//         bool is_node_obstacle(Node n, const ObstacleDistanceGrid& distances);
//         bool is_node_goal(Node n, Node g);
//         bool is_in_list(Node n, std::vector<Node> list);
//         bool is_goal_valid(Node g, const ObstacleDistanceGrid& distances, const SearchParams& params);
//         bool is_in_map(Node n, const ObstacleDistanceGrid& distances);
//         float calc_h_cost(Node n, Node g);
//         float calc_g_cost(Node node, Node neigh);
//         void expand_node(Node n, const ObstacleDistanceGrid& distances);
//         robot_path_t extract_path(Node n, const ObstacleDistanceGrid& distances, robot_path_t path);
        
        
//     private:
 

        
    

        
// };

#endif // PLANNING_ASTAR_HPP
