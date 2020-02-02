#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    delta_rot1 = 0.0;
    delta_rot2 = 0.0;
    delta_trans = 0.0;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{

    pose_xyt_t lastodo;
    pose_xyt_t lastpose;

    //If there's no odometry, then we're nowhere, suppose at origin.
    if (odomTrace_.empty())
    {
        lastodo.utime = 0;
        lastodo.x = 0;
        lastodo.y = 0;
        lastodo.theta = 0;

    }
    else{
        lastodo = odomTrace_.back();
    }

    if (poseTrace_.empty())
    {
        lastpose.utime = 0;
        lastpose.x = 0;
        lastpose.y = 0;
        lastpose.theta = 0;

    }
    else{
        lastpose = poseTrace_.back();
    }

    pose_xyt_t odo;
    odo.utime = odometry.utime;
    odo.x = odometry.x;
    odo.y = odometry.y;
    odo.theta = odometry.theta;
    odomTrace_.addPose(odo);
    newsample_time = odometry.utime;

    delta_rot1 = std::atan2(odo.y - lastodo.y, odo.x- lastodo.x) - lastodo.theta;
    // delta_rot1 = wrap_to_pi(delta_rot1);
    delta_trans = sqrt(pow(odo.x - lastodo.x, 2) + pow(odo.y- lastodo.y, 2));
    delta_rot2 = odo.theta - lastodo.theta - delta_rot1;
    delta_rot1 = wrap_to_pi(delta_rot1);
    delta_rot2 = wrap_to_pi(delta_rot2);

    return true;


//     if (delta_trans > 0.001)
//         return true;
//     if (delta_rot1 > 0.0001)
//         return true;
//     if (delta_rot2 > 0.0001)
//         return true;
   
//     return false;
 }


particle_t ActionModel::applyAction(const particle_t& sample)
{

    particle_t new_sample;
    // Action model noise, localization only
    // double a1=0.01; // firework
    // double a2=5; // nice to add, for localization only
    // double a3=3; // nice to add , for localization only
    // double a4=0.0001; // explode on transition

    // // Action model noise, slam
    // double a1=0.01; // firework
    // double a2= 5*0.1; // nice to add, for localization only
    // double a3= 3*0.1; // nice to add , for localization only
    // double a4=0.0001; // explode on transition
    // Action model noise, slam (4.26)
    double a1= 0.01; // firework
    double a2= .5; // nice to add, for localization only
    double a3= 0.3; // nice to add , for localization only
    double a4=0.005; // explode on transition
    //noisy motioncommand (those guys with hat are noisy motion command)
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d1{0, a1*(delta_rot1*delta_rot1) + a2* (delta_trans*delta_trans)};
    std::normal_distribution<> d2{0, a3*(delta_trans*delta_trans)+ a4* ((delta_rot1*delta_rot1)+ (delta_rot2*delta_rot2))};
    std::normal_distribution<> d3{0, a1*(delta_rot2*delta_rot2) + a2* (delta_trans*delta_trans)};  
    
 
    double delta_rot1_hat = wrap_to_pi(delta_rot1 - d1(gen));
    double delta_trans_hat = delta_trans - d2 (gen);

    double delta_rot2_hat = wrap_to_pi(delta_rot2 - d3 (gen));
    if (fabs(delta_rot1_hat)>= 3.14159/2 && fabs(delta_rot2_hat)>= 3.14159/2)
    {
        delta_rot1_hat = wrap_to_pi(delta_rot1_hat + 3.14159);
        delta_trans_hat = -delta_trans_hat;
        delta_rot2_hat = wrap_to_pi(delta_rot2_hat + 3.14159);
    }
    // std::cout<<"rot1_hat, trans_hat, rot2_hat = "<<delta_rot1_hat<<"  "<< delta_trans_hat<<" "<<delta_rot2_hat<<"\n";
    // std::cout<<"delta_rot1_hat"<<delta_rot1_hat<<"\n";
    // std::cout<<"delta_trans_hat"<<delta_trans_hat<<"\n";
    // std::cout<<"delta_rot2_hat"<<delta_rot2_hat<<"\n";
    //update new sample pose based on motion command
    new_sample.parent_pose = sample.pose; // prior pose = pose
    // new_sample.parent_pose.y = sample.pose.y;
    // new_sample.parent_pose.theta = sample.pose.theta;
    new_sample.pose.x = sample.pose.x + delta_trans_hat* cos(sample.pose.theta + delta_rot1_hat);
    new_sample.pose.y = sample.pose.y + delta_trans_hat* sin(sample.pose.theta + delta_rot1_hat);
    new_sample.pose.theta = wrap_to_pi(sample.pose.theta + delta_rot1_hat + delta_rot2_hat);
    new_sample.weight = sample.weight;
    new_sample.pose.utime = newsample_time;
    // std::cout<<"new_sample.pose.x"<<new_sample.pose.x<<"\n";
    // std::cout<<"new_sample.pose.y"<<new_sample.pose.y<<"\n";
    //store the new atucal pose in the Posetrace
    pose_xyt_t pose;
    pose.utime = sample.pose.utime;
    pose.x = new_sample.pose.x;
    pose.y = new_sample.pose.y;
    pose.theta = wrap_to_pi(new_sample.pose.theta);
    // pose.weight = new_sample.weight;
    poseTrace_.addPose(pose);
    // std::cout<<"reach end\n";
    // need angle clamping!
    return new_sample;
}