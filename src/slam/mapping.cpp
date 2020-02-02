#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}

void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pre_pose, const pose_xyt_t& cur_pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    // std::cout<<"currentPose_x"<<currentPose_.x<<"\n";
    // for (int i=0; i<scan.num_ranges; i++)
    MovingLaserScan  Correctedlaser(scan, pre_pose, cur_pose,1);
    for (int i=0; i<Correctedlaser.size(); i+=1)
    {
        // double z = scan.ranges[i];
        // double theta = -scan.thetas[i];

        // rayCastBreshenham(pose, z, theta, map);
        adjusted_ray_t curr_ray = Correctedlaser.at(i);
        double z = curr_ray.range;
        double theta = curr_ray.theta;
        pose_xyt_t pose;
        pose.x = curr_ray.origin.x;
        pose.y = curr_ray.origin.y;
        pose.theta = theta;
        rayCastBreshenham(pose, z, theta, map);
    }
}

void Mapping::rayCastBreshenham(const pose_xyt_t& pose, double z, double theta, OccupancyGrid& map)
{
    Point<int> map_start;
    Point<int> map_pos;
    Point<int> map_end;
    Point<double> start_point;
    Point<double> end_point;
    // theta += pose.theta;

    start_point.x = pose.x;
    start_point.y = pose.y;
    // end point is z away from start at angle theta
    end_point.x = start_point.x + z*cos(theta);
    end_point.y = start_point.y + z*sin(theta);

    map_start = global_position_to_grid_cell(start_point, map);
    map_end = global_position_to_grid_cell(end_point, map);

    int dx = abs(map_end.x - map_start.x);
    int dy = abs(map_end.y - map_start.y);
    int sx = map_start.x < map_end.x ? 1 : -1;
    int sy = map_start.y < map_end.y ? 1 : -1;
    int err = dx - dy;
    map_pos = map_start;

    while (map_pos.x != map_end.x || map_pos.y != map_end.y)
    {
        auto& odds = map(map_pos.x, map_pos.y);
        if (odds > -128 + kMissOdds_)
        {        
            odds -= kMissOdds_;
        }
        else
        {
            odds = -128;
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

    // hit grid at end of map
    auto& odds = map(map_pos.x, map_pos.y);
    if (odds < 127 - kHitOdds_)
    {
        odds += kHitOdds_;
    }
    else
    {
        odds = 127;
    }
}    