#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <common/point.hpp>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <cstdlib>
#include <unistd.h>
#include <string.h>
#include <map>

// initial declarations
typedef Point<int> cell_t;

   struct Node {
            cell_t cell;
            // struct Node* parent;
            // std::shared_ptr<struct Node> parent;
            float g_cost = 0.0;
            float h_cost = 0.0;
            float f_cost = 0.0;
            // float get_f_cost() = get_get_f_cost();

            // float get_f_cost() const
            // {   
                
            //     return g_cost + h_cost;
            // }
            // bool operator<(const Node& rhs) const
            // {
            //     return get_f_cost() < rhs.get_f_cost();
            // }

            // bool operator>(const Node& rhs) const
            // {
            //     return get_f_cost() > rhs.get_f_cost();
            // }
            // bool operator!=(const Node& rhs) const
            // {
            //     return (cell.x != rhs.cell.x) || (cell.y != rhs.cell.y)  || (get_f_cost() != rhs.get_f_cost()) || (g_cost != rhs.g_cost) || (h_cost != rhs.h_cost);
            // } 

            // bool operator==(const Node& rhs) const
            // {
            //     return (cell.x == rhs.cell.x) && (cell.y == rhs.cell.y) && (get_f_cost() == rhs.get_f_cost()) && (g_cost == rhs.g_cost) && (h_cost == rhs.h_cost);
            // }
            // float get_f_cost() const
            // {   
                
            //     return g_cost + h_cost;
            // }
            bool operator<(const Node& rhs) const
            {
                return f_cost < rhs.f_cost;
            }

            bool operator>(const Node& rhs) const
            {
                return f_cost > rhs.f_cost;
            }
            bool operator!=(const Node& rhs) const
            {
                return (cell.x != rhs.cell.x) || (cell.y != rhs.cell.y)  || (f_cost!= rhs.f_cost) || (g_cost != rhs.g_cost) || (h_cost != rhs.h_cost);
            } 

            bool operator==(const Node& rhs) const
            {
                return (cell.x == rhs.cell.x) && (cell.y == rhs.cell.y) && (f_cost == rhs.f_cost) && (g_cost == rhs.g_cost) && (h_cost == rhs.h_cost);
            }
        };

    bool path_found = false;
    Node Start_node;
    Node Goal_node;
    std::priority_queue<Node,std::vector<Node>,std::greater<Node>> open_list;
    std::vector<Node> closed_list;
    std::vector<Node> searched_list;
    std::map <cell_t, cell_t> parent_map;
    const int xDeltas[8] = {1, -1, 0,  0, 1, 1, -1, -1};
    const int yDeltas[8] = {0,  0, 1, -1, 1, -1, 1, -1};   

// function to check if node is free

bool is_node_free(Node& n, const ObstacleDistanceGrid& distances) {
    if (distances(n.cell.x, n.cell.y) == 100)
    {
        return true;
    }
    return false;
}

// function to check if node is obstacle

bool is_node_obstacle(Node& n, const ObstacleDistanceGrid& distances) {
    if (distances(n.cell.x, n.cell.y) == 0)
    {
        return true;
    }
    return false;
}

bool is_node_equal(Node& n1, Node& n2){
    if((n1.cell.x == n2.cell.x) && (n1.cell.y == n2.cell.y)) {
        return true;
    }
    return false;
}

bool is_cell_equal(cell_t& c1, cell_t& c2){
    if((c1.x == c2.x) && (c1.y == c2.y)) {
        return true;
    }
    return false;
}
//function to check if node is destination

bool is_node_goal(Node& n, Node& g) {
    if (n.cell.x == g.cell.x && n.cell.y == g.cell.y){
        return true;
    }
    return false;
}

bool is_in_list(Node& n, std::vector<Node>& list) {
   if(std::find(list.begin(), list.end(), n) != list.end())
   {
       return true;
   }
   return false;
}

bool is_goal_valid(Node& g, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    if(distances.isCellInGrid(g.cell.x, g.cell.y))
    {
        return distances(g.cell.x, g.cell.y) > params.minDistanceToObstacle;
    }
    return false;
}

bool is_node_safe(Node& n, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    if(distances.isCellInGrid(n.cell.x, n.cell.y))
    {
        return distances(n.cell.x, n.cell.y) > params.minDistanceToObstacle;
    }
    return false;
}

bool is_safe_cell(int x, int y, double robotRadius, const ObstacleDistanceGrid& distances)
{
    // Search a circular region around (x, y). If any of the cells within the robot radius are occupied, then the
    // cell isn't safe.
    const int kSafeCellRadius = std::lrint(std::ceil(robotRadius * distances.cellsPerMeter()));
    
    for(int dy = -kSafeCellRadius; dy <= kSafeCellRadius; ++dy)
    {
        for(int dx = -kSafeCellRadius; dx <= kSafeCellRadius; ++dx)
        {
            // Ignore the corners of the square region, where outside the radius of the robot
            if(std::sqrt(dx*dx + dy*dy) * distances.metersPerCell() > robotRadius)
            {
                continue;
            }
            
            // If the odds at the cells are greater than 0, then there's a collision, so the cell isn't safe
            if(distances(x + dx, y + dy) == 0)
            {
                return false;
            }
        }
    }
    
    // The area around the cell is free of obstacles, so all is well
    return true;
}

bool is_in_map(Node& n, const ObstacleDistanceGrid& distances) {
    int width = distances.widthInCells();
    int height = distances.heightInCells();
    if ((n.cell.x >= 0) && (n.cell.y >=0) && (n.cell.x < width) && (n.cell.y < height))
    {
        return true;
    }
    return false;
}
// function to calculate h cost
float calc_h_cost(Node& n, Node& g) {
    float x = std::abs(n.cell.x - g.cell.x);
    float y = std::abs(n.cell.y - g.cell.y);
   return 2* std::sqrt(x*x + y*y);
    
}

// function to calculate g cost
    float calc_g_cost(Node& to_node, Node& from_node) {
    float x = std::abs(to_node.cell.x - from_node.cell.x);
    float y = std::abs(to_node.cell.y - from_node.cell.y);
    return from_node.g_cost+std::sqrt(x*x + y*y);
}

// float get_obstacle_cost(cell_t& c, const ObstacleDistanceGrid& distances)
float calc_f_cost(Node& n, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    float obstacle_cost = 0.0;
    if (distances(n.cell.x, n.cell.y) > params.minDistanceToObstacle && 
        distances(n.cell.x, n.cell.y) < params.maxDistanceWithCost) {
            obstacle_cost = pow(params.maxDistanceWithCost - distances(n.cell.x, n.cell.y), params.distanceCostExponent);
    }
    return n.g_cost + n.h_cost + obstacle_cost;
}

void print_path(robot_path_t path){
    // std::cout<<"printing path"<<std::endl;
    for (int i = 0; i < path.path.size(); i++){
        std::cout << path.path.at(i).x << " "<< path.path.at(i).y<< std::endl;
    }
    // std::cout<<"path printing done"<<std::endl;
}


void expand_node(Node& n, const ObstacleDistanceGrid& distances, const SearchParams& params) {
     bool goal_matched = false;
    
    for(int i = 0; i < 8; i++){
        
            int x = n.cell.x + xDeltas[i];
            int y = n.cell.y + yDeltas[i];
            Node neighbour;
          
            auto it = std::find_if(searched_list.begin(), searched_list.end(), [x, y](const Node& n){return n.cell.x == x && n.cell.y == y;});
           
            if (it == searched_list.end())
            {
                neighbour.cell.x = x;
                neighbour.cell.y = y;
            }
            else 
            {
                 neighbour = *it;
            }
            
            if ((!is_in_list(neighbour,closed_list)) && is_in_map(neighbour,distances) && (!is_node_obstacle(neighbour,distances))) {
                if(!is_in_list(neighbour,searched_list)) {
                    neighbour.g_cost = calc_g_cost(neighbour, n);
                    neighbour.h_cost = calc_h_cost(neighbour, Goal_node);
                    neighbour.f_cost = calc_f_cost(neighbour, distances, params);
                    if(parent_map.find(neighbour.cell)==parent_map.end()) {
                        parent_map.insert(std::pair<cell_t,cell_t>(neighbour.cell, n.cell));
                    }
                    else {
                        parent_map[neighbour.cell] = n.cell;
                    }
                    if(is_safe_cell(neighbour.cell.x, neighbour.cell.y,params.minDistanceToObstacle, distances)){
                        open_list.push(neighbour);
                    }
                    searched_list.push_back(neighbour);
                }
                else if(neighbour.g_cost > calc_g_cost(neighbour, n)) {
                    neighbour.g_cost = calc_g_cost(neighbour, n);
                    neighbour.h_cost = calc_h_cost(neighbour, Goal_node);
                    neighbour.f_cost = calc_f_cost(neighbour, distances, params);
                    if(parent_map.find(neighbour.cell)==parent_map.end()) {
                        parent_map.insert(std::pair<cell_t,cell_t>(neighbour.cell, n.cell));
                    }
                    else {
                        parent_map[neighbour.cell] = n.cell;
                    }
                    if(is_safe_cell(neighbour.cell.x, neighbour.cell.y,params.minDistanceToObstacle, distances)){
                        open_list.push(neighbour);
                    }

                }
            }
            if(is_node_goal(neighbour, Goal_node)){
                goal_matched = true;
                break;
            }
    }
   
}

robot_path_t extract_path(cell_t& c, const ObstacleDistanceGrid& distances, pose_xyt_t start) {
    robot_path_t path;
    path.utime = start.utime;
    // Node current_node = n;
    // std::cout << "printing start coordinates : "<<Start_node.cell.x<<" "<<Start_node.cell.y<<std::endl;
    while(!is_cell_equal(c,Start_node.cell)){
    // std::cout<<"in while of extract path"<<std::endl;
    // std::cout<<"n node coordinates: "<<n.cell.x << " "<< n.cell.y<<std::endl; 
        pose_xyt_t pose;
        Point <double>grid_point;
        // grid_point.x = (double)n.cell.x;
        // grid_point.y = (double)n.cell.y;
        grid_point.x = (double)c.x;
        grid_point.y = (double)c.y;
        Point <double> global_point;
        global_point = grid_position_to_global_position(grid_point,distances);
        pose.x = global_point.x;
        pose.y = global_point.y;
        pose.theta = std::atan2(pose.y, pose.x);
        // strcpy(n, n.parent);
        c = parent_map[c];
        //  n = *n.parent;
        path.path.push_back(pose);
        // std::cout<<"nx,ny="<<n.cell.x<<","<<n.cell.y<<std::endl;
        // std::cout<<"path_length:"<<path.path.size()<<std::endl;
            // if(!is_node_equal(n,Start_node)){
            //     n = *n.parent;
            //     path.path.push_back(pose);
            // }        
    
    }
    // std::cout<<"End Path while\n";
    // print_path(path);
    std::reverse(path.path.begin(), path.path.end());
    // std::cout<<"End reverse\n";
    path.path_length = path.path.size();
    // std::cout<<"End set path length\n";
    // std::cout<< "returning from extract path"<<std::endl;
    
    return path;
}
// search path
  robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    closed_list.clear();
    searched_list.clear();
    open_list = std::priority_queue<Node,std::vector<Node>,std::greater<Node>>();
    path_found = false;
    std::map <cell_t, cell_t> parent_map;   
 
    //     
    
    if(start.x == goal.x && start.y == goal.y)
    {      
        robot_path_t path;
        path.utime = start.utime;
        path.path.push_back(start);
        path.path_length = path.path.size();
        return path; 
    }

    Point <double> start_pose;
    start_pose.x = start.x;
    start_pose.y = start.y;
    Point <double> goal_pose;
    goal_pose.x = goal.x;
    goal_pose.y = goal.y;


    // get start and goal cell
    cell_t start_cell = global_position_to_grid_cell(start_pose, distances);
    cell_t goal_cell = global_position_to_grid_cell(goal_pose, distances);
 
     // make start_node
    
    Start_node.cell = start_cell;
    
    // make goal_node
    
    Goal_node.cell = goal_cell;
    if(!is_in_map(Start_node, distances)){
        robot_path_t path ;
        return path;
    }

    if(is_node_obstacle(Start_node, distances)) {
        robot_path_t path ;
        return path;
    }

    Goal_node.h_cost = 0;
     
    Start_node.h_cost = calc_h_cost(Start_node,Goal_node);
    Start_node.f_cost = calc_f_cost(Start_node,  distances, params);
 

    open_list.push(Start_node);

    //check if valid goal
    std::cout<< "start pose: "<<start.x << " "<< start.y << std::endl;
    std::cout<<"start cell: "<<Start_node.cell.x<<" "<<Start_node.cell.y<<std::endl;
    std::cout<< "goal pose: "<<goal.x << " "<< goal.y << std::endl;
    std::cout<<"goal cell: "<<Goal_node.cell.x<<" "<<Goal_node.cell.y<<std::endl;

     while(!open_list.empty()) {
        Node current_node = open_list.top();
           open_list.pop();
        closed_list.push_back(current_node);
        // current_node = open_list.top();
        if(is_node_goal(current_node, Goal_node)) {
            path_found = true;
            // robot_path_t final_path = extract_path(current_node, distances, start);
            robot_path_t final_path = extract_path(current_node.cell, distances, start);
            
            return final_path;
        }
       
       
        // std::cout<<" Going From ("<<Start_node.cell.x <<","<<Start_node.cell.y<<")";
        // std::cout<<" to ("<<Goal_node.cell.x <<","<<Goal_node.cell.y<<")"<<std::endl;
        // std::cout<<" current node coordinates "<<current_node.cell.x <<" "<<current_node.cell.y<<std::endl;
        // std::cout<<"fcost : "<<current_node.get_f_cost()<<" gcost: "<<current_node.g_cost<<" hcost : "<<current_node.h_cost<<std::endl;
        // std::cout<<"\n"<<std::endl;
        // usleep(100000);
 
        expand_node(current_node, distances, params);
     
      } 
}




//    while(!open_list.empty()) {
//    Node current_node = open_list.top();
       
       
//         while(!is_node_goal(current_node,Goal_node)) {
//             // current_node = open_list.top();
            
//             // 
//             // std::cout<<"in while"<<std::endl;
            
//             closed_list.push_back(current_node);
            
            
//             std::cout<<"Expanding\n";
//             // i++;
//             expand_node(current_node, distances);
//             std::cout<<"Done Expanding\n";
            
//         //  }  
//             std::cout<<"Top\n";         
//             current_node = open_list.top();
//             std::cout<<"Pop\n";  
//             open_list.pop();
//         //  
//             std::cout<<" Going From ("<<Start_node.cell.x <<","<<Start_node.cell.y<<")";
//             std::cout<<" to ("<<Goal_node.cell.x <<","<<Goal_node.cell.y<<")"<<std::endl;
//             std::cout<<" current node coordinates "<<current_node.cell.x <<" "<<current_node.cell.y<<std::endl;
//             std::cout<<"fcost : "<<current_node.get_f_cost()<<" gcost: "<<current_node.g_cost<<" hcost : "<<current_node.h_cost<<std::endl;
//             std::cout<<"\n"<<std::endl;
//             usleep(10000);
           
//             if(is_node_goal(current_node, Goal_node)) {
//                 std::cout <<" path found" << std::endl;
//                 path_found = true;
//             }
//         }
//         std::cout<<"extract path\n";
//         robot_path_t final_path = extract_path(current_node, distances, path);
//         std::cout<<"path extracted\n";
//         // std::cout<<"returning final path"<<std::endl;
//         // print_path(final_path);

//         return final_path;
//    }


   
    





