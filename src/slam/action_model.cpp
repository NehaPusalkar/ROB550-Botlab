#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

// float get_stddev(double mean, double variance) {
    
// }
ActionModel::ActionModel(void)
: a1_(12.5f)
, a2_(100.8f)
, a3_(0.5f)
, a4_(0.025f)
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
    
    double dir = 1.0;
    if(std::abs(translation_) < 0.0001){
        rotation1_ = 0.0f;

    }
    if(std::abs(rotation1_) > M_PI/2.0){
        rotation1_ = angle_diff(M_PI,rotation1_);
        dir = -1.0;
    }
    rotation2_ = angle_diff(dtheta,rotation1_);
    stddev_rotation1_ = a1_ * pow(rotation1_,2) + a2_ * pow(translation_,2);
    stddev_rotation2_ = a1_ * pow(rotation2_,2) + a2_ * pow(translation_,2);
    stddev_translation_ = a3_ * pow(translation_,2) + a4_ * pow(rotation1_,2) + a4_ * pow(rotation2_,2);

    // stddev_rotation1_ = k1_ * std::abs(rotation1_);
    // stddev_translation_ = k2_ * std::abs(translation_);
    // stddev_rotation2_ = k1_ * std::abs(rotation2_);
    // stddev_rotation1_ = a1_ * pow(rotation1_,2) ;
    // stddev_rotation2_ = a1_ * pow(rotation2_,2);
    // stddev_translation_ = a3_ * pow(translation_,2);
   
    if((dX != 0.0) || (dY != 0.0) || (dtheta != 0.0)) {
        robot_moved_ = true;
        // return true;
    }
    else {
        robot_moved_ = false;
    }
    prv_odo_ = odometry;
    translation_ = dir * translation_;
    utime_ = odometry.utime;
       return robot_moved_;
    

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
        
        // double epsilon1 = std::normal_distribution<>(0, stddev_rotation1_)(numgen_);
        // double epsilon2 = std::normal_distribution<>(0, stddev_translation_)(numgen_);
        // double epsilon3 = std::normal_distribution<>(0, stddev_rotation2_)(numgen_);
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
