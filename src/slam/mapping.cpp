#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>

#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (!initialized_)
    {
        previousPose_ = pose;
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);
    initialized_= true;
    previousPose_ = pose;
    for(auto& ray : movingScan)
    {
        Point<float>ray_start_in_grid = global_position_to_grid_position(ray.origin, map); 
        Point <int> ray_end;
        ray_end.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + ray_start_in_grid.x);
        ray_end.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + ray_start_in_grid.y);
        scoreEndpoint(ray, map);
        Point<int>ray_start = global_position_to_grid_cell(ray.origin, map);
        
       
        //breshenham's algorithm
        int x1 = ray_end.x;
        int y1 = ray_end.y;
        int x0 = ray_start.x;
        int y0 = ray_start.y;
        int dx = fabs(x1 - x0);
        int dy = fabs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x = x0;
        int y = y0;

        while (x != x1 || y != y1) 
        {
            updateOdds(x,y, map);
            int e2 = 2 * err;
            if(e2 >= -dy)
            {
                err -= dy;
                x += sx;
            }

            if(e2 <= dx) 
            {
                err += dx;
                y += sy;
            }
        }
    }
    


    //for(auto& ray : movingScan)
    // {
    //     scoreRay(ray, map);
    // }

}

void Mapping :: scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range <=kMaxLaserDistance_)
    {
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)) {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }

    }
}

void Mapping :: increaseCellOdds(int x, int y, OccupancyGrid& map){
    if (!initialized_){
        //do nothing
    }
    //current cell not at max
    else if (std :: numeric_limits<CellOdds>:: max() - map(x,y) > kHitOdds_){
       map(x,y) += kHitOdds_;
    }
    //current cell already saturated
    else
    {
       map(x,y) = std ::numeric_limits<CellOdds>::max();
    }
}


void Mapping :: updateOdds(int x, int y, OccupancyGrid& map) {
    if (!initialized_){
        //do nothing
    }
     else if (fabs(std :: numeric_limits<CellOdds>:: min() - map(x,y)) > kMissOdds_)
     {
       map(x,y) -= kMissOdds_;
    }
      else
    {
       map(x,y) = std ::numeric_limits<CellOdds>::min();
    }
}