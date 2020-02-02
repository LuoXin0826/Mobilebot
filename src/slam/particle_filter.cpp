#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cmath>
#include <common/angle_functions.hpp>
#include <cassert>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose, const OccupancyGrid&   map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    // initialize pose
    std::vector<pose_xyt_t>posteriorPose_;
    double N_x = 0.04; //0.1
    double N_y = 0.04; //0.1
    double N_thets = 0.001;
    // initialize particles based on normal distribution
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<> dist_x(0.0, N_x);
    std::normal_distribution<> dist_y(0.0, N_y);
    std::normal_distribution<> dist_theta(0.0, N_thets);
    // std::uniform_real_distribution<> dist_x(-N_x, N_x);
    // std::uniform_real_distribution<> dist_y(-N_y, N_y);
    // std::uniform_real_distribution<> dist_theta(-N_thets, N_thets);
    // -------------------initialize from a given pose
    // for (int i = 0; i < kNumParticles_; i++)
    // {

    //     posteriorPose_.push_back(pose);
    //     posteriorPose_[i].x = dist_x(gen) + pose.x;
    //     posteriorPose_[i].y = dist_y(gen) + pose.y;
    //     // posteriorPose_[i].theta = pose.theta;
    //     posteriorPose_[i].theta = dist_theta(gen)+pose.theta;

    //     posterior_[i].weight = 1.0 / kNumParticles_;

    //     posterior_[i].pose.x = posteriorPose_[i].x;
    //     posterior_[i].pose.y = posteriorPose_[i].y;
    //     posterior_[i].pose.theta = posteriorPose_[i].theta;

    //     // std::cout<<posteriorPose_[i].x<<"\n";
    // }
    // -----------------------if we don't know where we are----------------------
    std::vector<Point<int>> unoccupied_pts;
    unsigned int n_free = 0;
    Point<int> free_point(0,0);
    for (int i = 0; i < map.widthInCells(); ++i)
    {
        for (int j = 0; j < map.heightInCells(); ++j)
        {
            double odds = map(i, j);
            if (odds > 100)
            {
                free_point.x = i;
                free_point.y = j;
                // Point<int> free_point(i,j); 
                // freepose_xy = grid_position_to_global_position(free_point, map);
                // freepose.x = freepose_xy.x;
                // freepose.y = freepose_xy.y;
                // freepose.theta = dist_theta(gen);
                // posteriorPose_.push_back(freepose);
                unoccupied_pts.push_back(free_point);
                n_free += 1;
            }
        }
    }
    int gap = n_free / kNumParticles_;
    Point<double> freepose_xy; 
    pose_xyt_t freepose;
    int count_free = 0;

    for (unsigned int i = 0; i < n_free; i += gap)
    {
        free_point = unoccupied_pts[i];
        freepose_xy = grid_position_to_global_position(free_point, map);
        freepose.x = freepose_xy.x;
        freepose.y = freepose_xy.y;
        freepose.theta = dist_theta(gen);
        posteriorPose_.push_back(freepose);
        count_free += 1;
    }
    int n_left = kNumParticles_-count_free;
    if (n_left > 0)
    {
        for (int i = 0; i < n_left; i += 1)
        {
            free_point = unoccupied_pts[i+1+gap*i];
            freepose_xy = grid_position_to_global_position(free_point, map);
            freepose.x = freepose_xy.x;
            freepose.y = freepose_xy.y;
            freepose.theta = dist_theta(gen);
            posteriorPose_.push_back(freepose);
        }
    } 
    for (int i = 0; i < kNumParticles_; i++)
    {
        posterior_[i].weight = 1.0 / kNumParticles_;
        posterior_[i].pose.x = posteriorPose_[i].x;
        posterior_[i].pose.y = posteriorPose_[i].y;
        posterior_[i].pose.theta = posteriorPose_[i].theta;
        // robot_path_t path;
        // path.path.push_back()
    }


}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    // bool flag1 = 0;
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    // add message to see !!
    // std::cout<< "hasRobotMoved" << hasRobotMoved;

    if(hasRobotMoved)
    {
        // std::cout<< "robot moved\n";
        //---------original sequence--------------
        auto prior = resamplePosteriorDistribution();
        // for (unsigned int i = 0; i < prior.size(); ++i)
        // {
        //     std::cout<<"prior i x = "<<prior[i].pose.x<<"y = "<<prior[i].pose.y<<"\n";
        // }        
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // for (unsigned int i = 0; i < posterior_.size(); ++i)
        // {
        //     std::cout<<"posterior_ i weight = "<<posterior_[i].weight<<"\n";
        // }          
        posteriorPose_ = estimatePosteriorPose(posterior_);
        




        // auto proposal = computeProposalDistribution(posterior_);
        // std::cout<< "succeed computeProposalDistribution\n";
        
        // std::cout<< "succeed computeNormalizedPosterior\n"; 
        // std::cout<< "succeed computeNormalizedPosterior\n";      
        
        // std::cout<<"odometry x "<<odometry.x<<"\n";
        // std::cout<< " estimatePosteriorPose x "<<posteriorPose_.x<<"\n"; 
        // std::cout<< " estimatePosteriorPose y "<<posteriorPose_.y<<"\n"; 
        // std::cout<< "succeed estimatePosteriorPose\n"; 
        // auto prior = resamplePosteriorDistribution();
        // auto posterior_ = resamplePosteriorDistribution();
        // std::cout<< "succeed resamplePosteriorDistribution\n";
        //-----------------------change the sequence----------
        // auto proposal = computeProposalDistribution(posterior_);
        // for (unsigned int i = 0; i < proposal.size(); ++i)
        // {
        //     std::cout<<"proposal i x = "<<proposal[i].pose.x<<"weight = "<<proposal[i].weight<<"\n";
        // }
        // posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // posteriorPose_ = estimatePosteriorPose(posterior_);
        // auto posterior_ = resamplePosteriorDistribution();


    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    // std::cout<<"enter particles\n";
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    // std::cout<<"enter resamplePosteriorDistribution\n";
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    // std::cout<<"enter resamplePosteriorDistribution\n";
    std::vector<particle_t> prior;
    // std::cout<<"initialize prior vector\n";
    // cummulative distribution
    // std::sort(posterior_.begin(), posterior_.end(), CompartParticleWeight);
    // std::vector<double> Weight;
    // // std::cout<<"initialize Weight vector\n";
    // // std::cout<< posterior_.size();
    // for (unsigned int i = 0; i < posterior_.size(); i++)
    // {
    //     // Weight[i] = posterior_[i].weight;
    //     // std::cout<<i;
    //     Weight.push_back(posterior_[i].weight);
    //     // std::cout<<"Weight size"<<Weight.size();
    //     // std::cout<<"assign Weight\n";
    // }
    // // std::cout<<"Weight size"<<Weight.size();
    // // std::cout<<"succeed Weight\n";
    // std::vector<double> cumsum_Weight_posterior;
    // double accm = 0;
    
    // for (unsigned int i = 0; i < posterior_.size(); i++)
    // {
    //     accm = std::accumulate(Weight.begin(), Weight.begin()+i, 0.0);
    //     // std::cout<<"accm"<<accm;
    //     cumsum_Weight_posterior.push_back(accm);
    // }
    // for (unsigned int i = 0; i < posterior_.size(); i++)
    // {
    //     cumsum_Weight_posterior.push_back( std::accumulate(Weight.begin(), Weight.begin()+i, 0) ); //is this okay??
    // }
    // randomly, uniformally, sample from the cummulative distribution of the probability distribution
    // *********************new one start from here********************
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<> dis(0.0, 1.0/posterior_.size());

    double r = dis(gen); // random number fron uniform distribution 
    // double r = ((double)rand()/(double)RAND_MAX)/((double)posterior_.size());
    // std::cout<<"r"<<r<<"\n";
    double U = 0; 
    // int j1 = 1;
    int i = 0;
    double c = posterior_[i].weight; // initialize partial sum
    // std::vector<double>::iterator j; 
    for (unsigned int m = 0; m < posterior_.size(); m++)
    {
        U = r + (double)m/((double)posterior_.size());
        // U = 0;
        // U = r + m/posterior_.size();
        while(U > c){
            i+= 1;
            c+=posterior_[i].weight;
        }
        // std::cout<<"r = "<<i <<"\n";
        prior.push_back(posterior_[i]);
        // prior[i].weight = 1/((double)posterior_.size());
        // prior[i].weight = 1./
        // rdN = dis(gen);
        // find j
        // j = std::upper_bound(cumsum_Weight_posterior.begin(), cumsum_Weight_posterior.end(), rdN); // is this okay??
        // j1 = (j - cumsum_Weight_posterior.begin());
        // prior.push_back(posterior_[j1]);
        // cumsum_Weight_prior+= prior[i].weight;
    }
    // std::cout<<"cumsum_Weight_prior"<<cumsum_Weight_prior;
    // normalization
    for (unsigned int i = 0; i < prior.size(); i++)
    {
        // std::cout<<"prior x, y"<<prior[i].pose.x<<prior[i].pose.y<<"\n";
        prior[i].weight = 1/prior.size();
    }
    // std::cout<<"prior.size"<<prior.size()<<"\n";
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    // std::cout<<"enter computeProposalDistribution\n";
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    for (unsigned int i = 0; i < prior.size(); i++)
    {
        proposal.push_back(actionModel_.applyAction(prior[i]) );
    }
    // std::cout<< "-----------applyAction----------------------\n";
    // for (unsigned int i = 0; i < prior.size(); i++)
    // {
    //     std::cout<<"proposal x"<<proposal[i].pose.x<<"\n";
    //     std::cout<<"proposal y"<<proposal[i].pose.y<<"\n";
    //     std::cout<<"proposal theta"<<proposal[i].pose.theta<<"\n";
    // }
    // std::cout<< "applyAction\n";
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    // std::cout<<"enter computeNormalizedPosterior\n";
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double likelihood_sum = 0;
    for (unsigned int i = 0; i < proposal.size(); i++)
    {
        posterior.push_back(proposal[i]);
        //*******************************which one is right??
        posterior[i].weight = sensorModel_.likelihood(proposal[i], laser, map);// this this part correct??
        // posterior[i].weight = proposal[i].weight*sensorModel_.likelihood(proposal[i], laser, map);// this this part correct??
        likelihood_sum += posterior[i].weight;

    }

    // normalize weight-----
    for (unsigned int i = 0; i < posterior.size(); i++)
    {
        posterior[i].weight /= likelihood_sum;
        // std::cout<<"posterior x, y"<<posterior[i].pose.x<<posterior[i].pose.y<<"\n";
        // std::cout<<"posterior weight"<<posterior[i].weight<<"\n";
        // if (posterior[i].weight > 0.9)
        // {
            // std::cout<<"posterior weight"<<posterior[i].weight<<"\n";
        // }
        
    }
    return posterior;
}



pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    // std::cout<<"enter estimatePosteriorPose\n";
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;

    // use max-weight pose for the update
    // sort posterior by weight
    // std::sort(posterior.begin(), posterior.end(), CompartParticleWeight());
    // std::sort(posterior.begin(), posterior.end(),
    //     [](particle_t const &a, particle_t const &b) { return a.weight < b.weight; });
    // pose.x = posterior.back().pose.x;
    // pose.y = posterior.back().pose.y;
    // pose.theta = posterior.back().pose.theta;
    // double total_weight = 0.0;
    // std::cout<<posterior.size();
    double temp_x = 0.0;
    double temp_y = 0.0;
    double temp_theta = 0.0;
    // std::cout<<"------------------posterior-------\n";
    for (unsigned int i = 0; i < posterior.size(); i++)
    {
        // std::cout<<"Weight : "<<posterior[i].weight<<"\n";
        // std::cout<<"Pose x : "<<posterior[i].pose.x<<"\n";
        temp_x = temp_x + posterior[i].pose.x * posterior[i].weight;
        temp_y = temp_y + posterior[i].pose.y * posterior[i].weight;
        temp_theta = temp_theta + posterior[i].pose.theta * posterior[i].weight;
        // pose.x += posterior[i].pose.x * posterior[i].weight;
        // std::cout<<"Pose x : "<<pose.x<<"\n";
        // pose.y += posterior[i].pose.y * posterior[i].weight;
        // pose.theta += posterior[i].pose.theta * posterior[i].weight;
        // total_weight += posterior[i].weight;
    }
    pose.x = temp_x;
    pose.y = temp_y;
    // pose.theta = temp_theta;
    pose.theta = wrap_to_pi(temp_theta);
    // pose.x /= total_weight;
    // pose.y /= total_weight;
    // pose.theta /= total_weight;
    // std::cout<<"pose x"<<pose.x<<"pose y"<<pose.y<<"\n";
    // if(pose.theta < -M_PI)
    // {
    //     for(; pose.theta < -M_PI; pose.theta += 2.0*M_PI);
    // }
    // else if(pose.theta > M_PI)
    // {
    //     for(; pose.theta > M_PI; pose.theta -= 2.0*M_PI);
    // }
    // poseTrace_.addPose(pose);
    // std::cout<<"pose x"<<pose.x<<"\n";
    // std::cout<<"pose y"<<pose.y<<"\n";
    // std::cout<<"total_weight"<<total_weight<<"\n";
    return pose;
}

// bool ParticleFilter::CompartParticleWeight(const particle_t& p1, const particle_t& p2)
// {
//     return (p1.weight < p2.weight);
// }
