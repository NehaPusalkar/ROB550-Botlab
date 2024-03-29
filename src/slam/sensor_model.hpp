#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP
#include<common/point.hpp>
class  lidar_t;
class  OccupancyGrid;
struct particle_t;
struct adjusted_ray_t;
/**
* SensorModel implement a sensor model for computing the likelihood that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized likelihood of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map)
*
* likelihood() computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{
public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-likelihood will be calculated
    * \param    scan                Laser scan to use for estimating log-likelihood
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    double likelihood( particle_t& particle, const lidar_t& scan, const OccupancyGrid& map);

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////
    bool initialized_;
    int odds_at_;
    // int odds_after_;
    // int odds_before_;
    float score_frac_;
    float cell_diag_;
    const float  kMaxLaserDistance_;
    Point<int> Bresenham(int x0, int y0, int x1, int y1);
    double scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map); 

};

#endif // SLAM_SENSOR_MODEL_HPP
