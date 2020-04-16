#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
:kMaxLaserDistance_(5.0f)
,odds_at_(30.0)
,score_frac_(0.75)
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
        // std::cout<<"after score"<<std::endl;
        scanScore += rayScore;
    }
    return scanScore;
   // return 1.0;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map) {
    // if(ray.range <=kMaxLaserDistance_)
    // {
        double ray_score = 0.0;
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayTip;

        rayTip.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayTip.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        Point<int>rayTip_cell = global_position_to_grid_cell(rayTip,map);

        int x = rayTip_cell.x;
        int y = rayTip_cell.y;
        Point<int> prv_cell;
        prv_cell.x = static_cast<int>(rayTip.x - cell_diag_ * std::cos(ray.theta));
        prv_cell.y = static_cast<int>(rayTip.y - cell_diag_ * std::sin(ray.theta));
        Point<int>prv_cell_grid = global_position_to_grid_cell(prv_cell,map);
       
        Point<int> next_cell;
        next_cell.x = static_cast<int>(rayTip.x + cell_diag_ * std::cos(ray.theta));
        next_cell.y = static_cast<int>(rayTip.y + cell_diag_ * std::sin(ray.theta));
        Point<int>next_cell_grid = global_position_to_grid_cell(next_cell,map);

        if (map.logOdds(x,y) > 0) // cell occupied
        {
            ray_score = odds_at_; 
        } 
        // prev cell coordinates
        
        else if (map.logOdds(prv_cell_grid.x,prv_cell_grid.y)>0) // check previous cell
        {
            ray_score = score_frac_ * odds_at_;

        }
        //next cell coordinates
        
        else if(map.logOdds(next_cell_grid.x,next_cell_grid.y)>0) // check next cell
        {
            ray_score = score_frac_ * odds_at_;
        }

        else //still no obstacle
        {
            ray_score = 0.1 * odds_at_;
        }

        return ray_score;
    // }
}