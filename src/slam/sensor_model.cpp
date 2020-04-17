#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
:kMaxLaserDistance_(5.0f)
,odds_at_(30.0)
,score_frac_(0.5)
,cell_diag_(0.0707)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
     if (!initialized_)
    {
        sample.parent_pose = sample.pose;
    }
    
    double scanScore = 0.0;
    double rayScore = 0.0;
    // std::cout<<"b4 scan"<<std::endl;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    // std::cout<<"after scan"<<std::endl;
    initialized_= true;
    sample.parent_pose = sample.pose;
    for(const auto& ray : movingScan)
    {
        // std::cout<<"b4 score"<<std::endl;
        rayScore=scoreRay(ray, map);
        //std::cout<<"ray score" << rayScore<<std::endl;
        // std::cout<<"after score"<<std::endl;
        scanScore += rayScore;
    }
    std::cout<<"scan score" << scanScore<<std::endl;
    return scanScore;
   // return 1.0;
}

Point<int> SensorModel::Bresenham(int x0, int y0, int x1, int y1) {
        int dx = std::fabs(x1 - x0);
        int dy = std::fabs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x = x0;
        int y = y0;
        int count = 1;
        while ((x != x1 || y != y1) && count > 0)
        {
            count--;
            //updateOdds(x,y, map);
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
            // std::cout<<"x = "<<x<<"y = "<<y<<std::endl;
        Point<int> result;
        result.x = x;
        result.y = y;
        return result;
        
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map) {

        double ray_score = 0.0;
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayTip;

        rayTip.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayTip.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        // Point<int>rayTip_cell = global_position_to_grid_cell(rayTip,map);
        int x = rayTip.x;
        int y = rayTip.y;
        

        //breshenham to get prev cell coordinates
        Point<int> prev = SensorModel::Bresenham(rayTip.x, rayTip.y, rayStart.x, rayStart.y);
        
        //breshenham to get next cell coordinates
        Point<int> raynewTip;
        raynewTip.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayTip.x);
        raynewTip.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayTip.y);
        Point<int> newp = SensorModel::Bresenham(rayTip.x, rayTip.y, raynewTip.x, raynewTip.y);

        // Point<int> prv_cell;
        // prv_cell.x = static_cast<int>(rayTip.x - cell_diag_ * std::cos(ray.theta)*map.cellsPerMeter());
        // prv_cell.y = static_cast<int>(rayTip.y - cell_diag_ * std::sin(ray.theta)* map.cellsPerMeter());
        // Point<int>prv_cell_grid = global_position_to_grid_cell(prv_cell,map);
       
        // Point<int> next_cell;
        // next_cell.x = static_cast<int>(rayTip.x + cell_diag_ * std::cos(ray.theta)*map.cellsPerMeter());
        // next_cell.y = static_cast<int>(rayTip.y + cell_diag_ * std::sin(ray.theta)*map.cellsPerMeter());
        // Point<int>next_cell_grid = global_position_to_grid_cell(next_cell,map);

        if (map.logOdds(x,y) > 100) // cell occupied
        {
            // std::cout<<"hit"<<std::endl;
            ray_score = odds_at_; 
        } 
        // prev cell coordinates
        
        else if (map.logOdds(prev.x,prev.y) > 100) // check previous cell
        {   
            // std::cout<<"prev cell"<<map.logOdds(prev.x,prev.y)<<std::endl;
            ray_score = score_frac_ * odds_at_;

        }
        //next cell coordinates
        
        else if(map.logOdds(newp.x,newp.y)>100) // check next cell
        {
            // std::cout<<"next cell"<<map.logOdds(newp.x, newp.y)<<std::endl;
            ray_score = score_frac_ * odds_at_;
        }

        else //still no obstacle
        {
            // std::cout<<"nowhere"<<std::endl;
            ray_score = 0.01 * odds_at_;
        }

        return ray_score;

}