#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <stdlib.h>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
double sampleWeight = 1.0 / kNumParticles_;
posteriorPose_ = pose;

std :: random_device rd;
std :: mt19937 generator(rd());
std :: normal_distribution<> dist(0.0,0.01);
    for(auto& p : posterior_){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator));
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
        // std::cout<<"initial pose, weight"<<p.pose.x << " "<<p.pose.y<<" "<<p.pose.theta<<" "<< p.weight<<std::endl;
        
            } 
    posterior_.back().pose = pose;
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        // std::cout<<"start"<<std::endl; 
        auto prior = resamplePosteriorDistribution();
        // std::cout<<"resampled"<<std::endl;
        auto proposal = computeProposalDistribution(prior);
        // std::cout<<"proposal done"<<std::endl;
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // std::cout<<"posterior done"<<std::endl;
        posteriorPose_ = estimatePosteriorPose(posterior_);
        // std::cout<<"got pose"<<std::endl;
    }
    
    posteriorPose_.utime = odometry.utime;
    //posteriorPose_ = estimatePosteriorPose(posterior_);
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior;
    double M_inv = 1.0 / kNumParticles_;
    double r = ((float)rand() / RAND_MAX)* M_inv;
    // std::cout<<"r="<<r<<std::endl;
    double c = posterior_[0].weight;
    int i = 0;
    for (int m = 0; m < kNumParticles_; m++) {
        double U = r + m * M_inv;
        while (U > c) {
            
            i++;

            if(i == posterior_.size()){
                i--;
                break;
            }
            c = c + posterior_[i].weight;
           
        }
        // std::cout<<"i = "<<i<<std::endl; 
        // prior.push_back(posterior_[i].pose)
         prior.push_back(posterior_[i]);
        // std::cout<<"posterior pose"<<posterior_[i].pose.x << " "<<posterior_[i].pose.y << " "<< posterior_[i].pose.theta<< std::endl; 
        // std::cout<<"prior pose"<<prior.back().pose.x << " "<<prior.back().pose.y << " "<< prior.back().pose.theta<< std::endl; 
    }
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for(auto& p : prior) {
        proposal.push_back(actionModel_.applyAction(p));
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    posterior = proposal;
    double weight = 0.0;
    double total_weight = 0.0;

    for (auto&p : posterior) {
        // std::cout<<"b4 likelihood"<<std::endl;
        // std::cout<<"pose"<<p.pose.x<< " " << p.pose.y << " "<<p.pose.theta<<std::endl;
        weight = sensorModel_.likelihood(p, laser, map);
        // std::cout<<"after likelihood"<<std::endl;
        // std::cout<<"weight"<<weight<<std::endl;
        p.weight = weight;
        
        total_weight += weight;
        // posterior.push_back(p); 
    }
    // std::cout<<"total_weight"<<total_weight<<std::endl;
    for (auto&p : posterior) {

        p.weight /= total_weight;
        // std::cout<<"particle weight = " << p.weight<<std::endl;
    }
    double total_from_particles = 0.0;
    for (auto&p : posterior) {
        total_from_particles += p.weight;
    }

    // std::cout<<"particle total"<<total_from_particles<<std::endl;
    // assert(false);
    
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    float final_x = 0.0;
    float final_y = 0.0;
    float theta = 0.0;
    float sin_theta = 0.0;
    float cos_theta = 0.0;
    for (auto&p : posterior) {
        // std::cout<<"pose: "<<p.pose.x<<" "<<p.pose.y<< " "<<p.pose.theta<<std::endl;
        final_x += p.weight * p.pose.x;
        final_y += p.weight * p.pose.y;
        sin_theta += p.weight * std ::sin(p.pose.theta);
        cos_theta += p.weight * std :: cos(p.pose.theta);
    }

    theta = std:: atan2(sin_theta,cos_theta);
    pose.x = final_x ;
    pose.y = final_y ;
    pose.theta = theta;
    // std::cout<<"final pose:" << pose.x<< " "<< pose.y<< " "<<pose.theta << std::endl;
    return pose;
}
