#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <lcmtypes/lidar_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/grid_utils.hpp>
#include <cmath>
#include <common/point.hpp>

SensorModel::SensorModel(void)
{
    ///////// Handle any initialization needed for your sensor model
    z_max = ZMAX;
    z_hit = ZHIT;
    z_rand = ZRAND;
    z_short = ZSHORT;
    sigma_hit = SIGMA_HIT;
    lambda_short = LAMBDA_SHORT;
    dtheta = D_THETA;
    max_range = MAX_RANGE;
}

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan  Correctedlaser(scan, sample.parent_pose, sample.pose,5);
    int q = 0;
    int p;

    for (int i=0; i<Correctedlaser.size(); i+=1)
    {
        adjusted_ray_t curr_ray = Correctedlaser.at(i);
        // get p - likelihood of each individual lidar reading 

        double z = curr_ray.range;
        double theta = curr_ray.theta;
        pose_xyt_t pose;
        pose.x = curr_ray.origin.x;
        pose.y = curr_ray.origin.y;
        pose.theta = theta;

        // z_star - the distance from pose to nearest object in map
        // double z_star = rayCastBreshenham(pose, -theta, map);
        double z_star = rayCastBreshenham(pose, theta, map);
        // double p_hit = getPhit(z, z_star, pose, map);
        // double p_short = getPshort(z, z_star, lambda_short);
        // double p_rand = getPrand(z);
        // double p_max = getPmax(z);

        // double p = z_hit * p_hit + z_short * p_short + z_rand * p_rand + z_max * p_max;
        // std::cout<<"p = "<<p<<"\n";
        // q = q*p;
        double thre = 0.05;
        
        // std::cout<<" z is: "<<z<<"\n";
        // std::cout<<" z_star is: "<<z_star<<"\n";
        // std::cout<<" difference is: "<<fabs(z-z_star)<<"\n";
        if(fabs(z-z_star)<=thre){
            // std::cout<<"im here 1 \n";
            p=-4+12;
        }
        else if(z<z_star-thre){
            // std::cout<<"im here 2 \n";
            p=-8+12;
        }
        else{
            p=-12+12;
        }
        q = q+p;
    }
    return q;
}

inline double SensorModel::rayCastBreshenham(const pose_xyt_t& pose, double theta, const OccupancyGrid& map)
{
    Point<int> map_start;
    Point<int> map_pos;
    Point<int> map_end;
    Point<double> curr_point;
    Point<double> start_point;
    double distance;

    start_point.x = pose.x;
    start_point.y = pose.y;

    map_start = global_position_to_grid_cell(start_point, map);
    // theta += pose.theta;

    // take map end to be some arbitrarily large distance to ensure it is off the map
    map_end.x = map_start.x + (int)10*cos(theta);
    map_end.y = map_start.y + (int)10*sin(theta);

    int dx = abs(map_end.x - map_start.x);
    int dy = abs(map_end.y - map_start.y);
    int sx = map_start.x < map_end.x ? 1 : -1;
    int sy = map_start.y < map_end.y ? 1 : -1;
    int err = dx - dy;
    map_pos = map_start;

    while (map.isCellInGrid(map_pos.x, map_pos.y))
    {
        
        if ((int)map.logOdds(map_pos.x, map_pos.y) > 100)
        {
            curr_point = grid_position_to_global_position(map_pos, map);
            distance = (double) sqrt((start_point.x - curr_point.x)*(start_point.x - curr_point.x) + (start_point.y - curr_point.y)*(start_point.y - curr_point.y));
            return distance;
        }
        
        int e2 = 2 * err;

        if (e2 >= -dy)
        {
            err -= dy;
            map_pos.x += sx;
        }
        if (e2 <= dy)
        {
            err += dx;
            map_pos.y += sy;
        }
    }
    return max_range;
}

inline double SensorModel::normalPdf(double z, double z_star)
{
    return exp(-pow(z-z_star,2) / (2*pow(sigma_hit,2)))/ sqrt(2 * M_PIl * pow(sigma_hit, 2));
}

inline double SensorModel::integrateSimpson(float a, float b, int n, double z_star)
{
    // Integrates a normal distribution between a and b by simpson method
    float dz = (b - a) / n;
    std::vector<double> z(n);
    for (int i=0; i<n; i++)
    {
        z[i] = a + dz * i;
    }

    double integral;

    integral = normalPdf(z[0], z_star);  

    for (int i=1; i<n-1; i++)
    {
        if (i % 2)
        {
            integral += 4 * normalPdf(z[i], z_star);
        }
        else
        {
            integral += 2 * normalPdf(z[i], z_star);
        }
    }

    integral += normalPdf(z.back(), z_star);

    return dz * integral / 3;
}



/* functions for computing probabilities */
// double SensorModel::getPhit(double z, double z_star, const pose_xyt_t &pose, const OccupancyGrid& map)
// {    
//     double p = 0.0;
//     if (z < z_max)
//     {
//         //normalisation coefficient
//         double eta = 1.0 / integrateSimpson(0.0, max_range, 100, z_star);

//         p = eta * normalPdf(z, z_star);
//     }
//     return p; 
// }

double SensorModel::getPhit(double z, double z_star, const pose_xyt_t& pose, const OccupancyGrid& map)
{    
    //normalisation coefficient
    double eta = 1.0 / integrateSimpson(0.0, max_range, 100, z_star);
    double p = eta * normalPdf(z, z_star);
    return p; 
}

double SensorModel::getPshort(double z, double z_star, double lambda)
{
    double p = 0.0;
    if (z < z_star)
    {
        // get normalisation coefficient 
        double eta = 1.0 / ( 1 - exp(- lambda * z_star));
        p = eta * lambda * exp(-lambda * z);
    } 
    return p;
}
double SensorModel::getPmax(double z)
{
    double p = 0.0;
    if (z >= max_range)
    {
        p = 1.0;
    }
    return p;
}

double SensorModel::getPrand(double z)
{
    double p = 0.0;
    if (z < max_range)
    {
        p = 1.0 / max_range;
    }
    return p;
}