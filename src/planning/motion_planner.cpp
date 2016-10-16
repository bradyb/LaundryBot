#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <mapping/occupancy_grid_utils.hpp>
#include <common/timestamp.h>

namespace eecs467
{

MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1.0;
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


maebot_path_t MotionPlanner::planPath(const maebot_pose_t& start, const maebot_pose_t& goal) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        maebot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);
        
        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams_);
}


bool MotionPlanner::isValidGoal(const maebot_pose_t& goal) const
{
    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);
    
    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    
    // A goal must be in the map for the robot to reach it
    return false;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}
    
} // namespace eecs467
