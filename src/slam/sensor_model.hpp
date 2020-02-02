#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP
#include<vector>
#include <common/point.hpp>
#define MAX_RANGE       12.0
#define ZMAX            0.05
#define ZRAND           0.05
#define ZSHORT          0.2
#define ZHIT            0.7
#define LAMBDA_SHORT    1.0
#define SIGMA_HIT       0.1
#define D_THETA         0.1

class  lidar_t;
class  OccupancyGrid;
class pose_xyt_t;
struct particle_t;

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
    double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map);

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////
    double rayCastBreshenham(const pose_xyt_t& pose, double theta, const OccupancyGrid& map);
    double integrateSimpson(float a, float b, int n, double z_star);
    double normalPdf(double z, double z_star);

    // get probabilities
    double getPmax(double z);
    double getPrand(double z);
    double getPshort(double z, double z_star, double lambda);
    double getPhit(double z, double z_star, const pose_xyt_t& pose, const OccupancyGrid& map);

    // class variables
    double lambda_short;
    double sigma_hit;
    double z_max;
    double z_short;
    double z_hit;
    double z_rand;
    double max_range;
    double dtheta;
};

#endif // SLAM_SENSOR_MODEL_HPP