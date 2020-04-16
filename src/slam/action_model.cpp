#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

// float get_stddev(double mean, double variance) {
    
// }
ActionModel::ActionModel(void)
: a1_(0.0001f)
, a3_(0.025f)
, initialised_(false)

{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialised_)
    {
        initialised_ = true;
        prv_odo_ = odometry;
    }
    
    double dX = odometry.x - prv_odo_.x;
    double dY = odometry.y - prv_odo_.y;
    double dtheta = angle_diff(odometry.theta, prv_odo_.theta);
    translation_ = std::sqrt(pow(dX,2) + pow(dY,2));
    rotation1_ = angle_diff(std::atan2(dY,dX),prv_odo_.theta);
    
    utime_ = odometry.utime;
    if(std::abs(translation_) < 0.0001f){
        rotation1_ = 0.0f;

    }
    if(std::abs(rotation1_ > M_PI/2.0)){
        rotation1_ = angle_diff(M_PI,rotation1_);
        translation_ = -translation_;
    }
    rotation2_ = angle_diff(dtheta,rotation1_);
    stddev_rotation1_ = a1_ * pow(rotation1_,2);
    stddev_rotation2_ = a1_ * pow(rotation2_,2);
    stddev_translation_ = a3_ * pow(translation_,2);
    prv_odo_ = odometry;
    if((dX != 0.0) || (dY != 0.0) || (dtheta != 0.0)) {
        robot_moved_ = true;
        return true;
    }
       return false;
    

    }


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    particle_t new_samp = sample;
    if(robot_moved_)
    {
        // std::cout<<"came here"<<std::endl;
        double rot1_hat = std::normal_distribution<>(rotation1_, stddev_rotation1_)(numgen_);
        double rot2_hat = std::normal_distribution<>(rotation2_, stddev_rotation2_)(numgen_);
        double trans_hat = std::normal_distribution<>(translation_, stddev_translation_)(numgen_);
        new_samp.pose.utime = utime_;
        new_samp.parent_pose = sample.pose;
        new_samp.pose.x += trans_hat * cos(sample.pose.theta + rot1_hat);
        new_samp.pose.y += trans_hat * sin(sample.pose.theta + rot1_hat);
        new_samp.pose.theta = wrap_to_pi(sample.pose.theta + rot1_hat + rot2_hat); 
        // std::cout <<"ended"<<std::endl; 
    }
    else {
       // new_samp = sample;
        new_samp.pose.utime = utime_;
        new_samp.parent_pose = sample.pose;
    }
    return new_samp;
}
