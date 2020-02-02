#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
, map()
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    for (int x=0;x<width_;++x) {
        for (int y=0;y<height_;++y) {
            compute_distance(x,y,map);
        }
    }
  
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}

void ObstacleDistanceGrid::compute_distance(const int x, const int y, const OccupancyGrid& map) {
    bool found = false;
    float min_dist = 1000000000.0;
    if (is_safe(x,y,map)) {
        this->distance(x,y)=0.0f;
        return;
    }
    for (int distance = 0;;distance++) {
        if(found) return;
        for (int x_dist=distance;x_dist>=0;--x_dist) {
            int y_dist = distance - x_dist;
            float cell_dist = sqrt(x_dist*x_dist+y_dist*y_dist);
            cell_dist = cell_dist * metersPerCell_;

            if (is_safe(x+x_dist, y+y_dist,map)) {
                if (cell_dist < min_dist) {
                    this->distance(x,y) = cell_dist;
                    found = true;
                }
            }
            if (is_safe(x-x_dist, y+y_dist,map)) {
                if (cell_dist < min_dist) {
                    this->distance(x,y) = cell_dist;
                    found = true;
                }
            }
            if (is_safe(x-x_dist, y-y_dist,map)) {
                if (cell_dist < min_dist) {
                    this->distance(x,y) = cell_dist;
                    found = true;
                }
            }
            if (is_safe(x+x_dist, y-y_dist,map)) {
                if (cell_dist < min_dist) {
                    this->distance(x,y) = cell_dist;
                    found = true;
                }
            }
        }
    }
}

bool ObstacleDistanceGrid::is_safe(int x, int y, const OccupancyGrid& map ) {
    if (map.isCellInGrid(x,y)) {
        return map(x,y) >= 0;
    }
    return true;
}
