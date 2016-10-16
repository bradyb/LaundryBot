#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/timestamp.h>
#include <mapping/occupancy_grid_utils.hpp>
#include <limits>
#include <map>
#include <queue>
#include <cassert>

namespace eecs467
{
    
typedef Point<int> cell_t;
typedef std::map<cell_t, cell_t> PredecessorMap;

    
struct SearchNode
{
    cell_t cell;
    cell_t parent;
    double costSoFar;
    double costToGo;
    
    double cost(void) const { return costSoFar + costToGo; }
    
    bool operator<(const SearchNode& rhs) const
    {
        // The node with higher cost has a lower priority
        return cost() > rhs.cost();
    }
};


typedef std::map<cell_t, SearchNode> CostMap;


double cost_to_go(cell_t cell, cell_t goal);
double cost_to_move(cell_t from, cell_t to, const ObstacleDistanceGrid& distances, const SearchParams& params);

bool is_lower_cost_path(const SearchNode& node, const CostMap& costs);
void expand_node(const SearchNode& node, 
                 cell_t goal,
                 const ObstacleDistanceGrid& distances, 
                 const SearchParams& params,
                 std::priority_queue<SearchNode>& searchQueue);

std::vector<maebot_pose_t> extract_path_from_predecessors(cell_t start,
                                                          cell_t goal, 
                                                          const PredecessorMap& predecessors,
                                                          const ObstacleDistanceGrid& grid);
cell_t find_next_vertex_in_path(cell_t start, const PredecessorMap& predecessors);
maebot_pose_t path_cell_to_pose(cell_t cell, cell_t previousVertex, const ObstacleDistanceGrid& grid);
    
    
maebot_path_t search_for_path(maebot_pose_t start, 
                              maebot_pose_t goal, 
                              const ObstacleDistanceGrid& distances,
                              const SearchParams& params)
{
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    
    SearchNode initialNode;
    initialNode.cell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    initialNode.parent = initialNode.cell;  // start cell is its own parent
    initialNode.costSoFar = 0.0;            // the robot hasn't traveled, so there can't be any cost accrued yet
    initialNode.costToGo = cost_to_go(initialNode.cell, goalCell);
    
    std::priority_queue<SearchNode> searchQueue;
    CostMap costs;
    PredecessorMap predecessors;
    
    searchQueue.push(initialNode);
    predecessors[initialNode.cell] = initialNode.cell;
    
    bool foundPath = false;
    
    while(!searchQueue.empty())
    {
        SearchNode nextNode = searchQueue.top();
        searchQueue.pop();
        
        // Only expand a node if its cost is lower than the previous best-cost path. If higher cost, then toss it out
        if(is_lower_cost_path(nextNode, costs))
        {
            costs[nextNode.cell] = nextNode;
            predecessors[nextNode.cell] = nextNode.parent;
            
            expand_node(nextNode, goalCell, distances, params, searchQueue);
        }
        
        // If the goal is popped, then stop the search. Technically a slightly shorter path might exist, but since
        // the step size is a single cell, the found path will be very close or the same as the actual optimal path
        if(nextNode.cell == goalCell)
        {
            foundPath = true;
            break;
        }
    }
    
    maebot_path_t path;
    path.utime = utime_now();
    
    // If a path was found, then extract it via the predecessors by working backward from the goal
    if(foundPath)
    {
        path.path = extract_path_from_predecessors(initialNode.cell, goalCell, predecessors, distances);
    }
    // Otherwise, set the path to just contain the start, per the spec
    else
    {
        path.path.push_back(start);
    }
    
    path.path_length = path.path.size();
    return path;
}


double cost_to_go(cell_t cell, cell_t goal)
{
    // Cost to go is just the as-the-crow-flies distance
    return distance_between_points(cell, goal);
}


double cost_to_move(cell_t from, cell_t to, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // cost to move is the distance moved and the cost of being in the new cell, based on its distance from obstacles
    double distanceCost = distance_between_points(from, to);
    
    float distToObstacle = distances(to.x, to.y);
    double obstacleCost = 0.0;
    
    // If the distance is below the minimum allowed, the cost in infinite
    if(distToObstacle < params.minDistanceToObstacle)
    {
        obstacleCost = 100;
    }
    // if the distance is less than the max distance with a cost, then determine the cost using the search params
    else if(distToObstacle < params.maxDistanceWithCost)
    {
        obstacleCost = std::pow(params.minDistanceToObstacle - distToObstacle, params.distanceCostExponent);
    }
    // Otherwise, far enough from an obstacle for there to be no cost
    
    return distanceCost + obstacleCost;
}


bool is_lower_cost_path(const SearchNode& node, const CostMap& costs)
{
    auto costIt = costs.find(node.cell);
    
    return (costIt == costs.end())      // if this is the first to reach this node, it must be the closest
        || (node.costSoFar < costIt->second.costSoFar); // or did it take less cost to reach than previous best path?
}


void expand_node(const SearchNode& node, 
                 cell_t goal,
                 const ObstacleDistanceGrid& distances, 
                 const SearchParams& params,
                 std::priority_queue<SearchNode>& searchQueue)
{
    // Use an 8-way search
    const int xDeltas[8] = { 1, -1, 0, 0, -1, -1, 1, 1 };
    const int yDeltas[8] = { 0, 0, 1, -1, -1, 1, -1, 1 };
    
    for(int n = 0; n < 8; ++n)
    {
        cell_t adjacentCell(node.cell.x + xDeltas[n], node.cell.y + yDeltas[n]);
        
        bool canGoThroughWall = (distances(node.cell.x, node.cell.y) < params.minDistanceToObstacle) 
            && (distances(node.cell.x, node.cell.y) > 0);

        if((adjacentCell != node.parent)  // path can't backtrack on itself
            && (distances.isCellInGrid(adjacentCell.x, adjacentCell.y)) // the cell must be in the grid
            && (canGoThroughWall || (distances(adjacentCell.x, adjacentCell.y) > params.minDistanceToObstacle))) // the cell can't be too close to the wall
        {
            SearchNode nextNode;
            nextNode.cell = adjacentCell;
            nextNode.parent = node.cell;
            nextNode.costSoFar = node.costSoFar + cost_to_move(node.cell, adjacentCell, distances, params);
            nextNode.costToGo = cost_to_go(adjacentCell, goal);
            
            searchQueue.push(nextNode);
        }
    }
}


std::vector<maebot_pose_t> extract_path_from_predecessors(cell_t start,
                                                          cell_t goal, 
                                                          const PredecessorMap& predecessors,
                                                          const ObstacleDistanceGrid& grid)
{
    // Ensure the start and goal are in the predecessors so the search will terminate
    assert(predecessors.find(goal) != predecessors.end());
    assert(predecessors.find(start) != predecessors.end());
    
    std::vector<maebot_pose_t> path;
    path.push_back(path_cell_to_pose(goal, goal, grid));
    
    cell_t edgeStart = goal;
    
    // Trace back one vertex at a time through the path. Finding the next vertex will follow the predecessors in a
    // straight line until a turn is found. Thus, the path is the minimum number of vertices needed to specify the lines
    // the robot should follow from start to goal
    while(edgeStart != start)
    {
        cell_t edgeEnd = find_next_vertex_in_path(edgeStart, predecessors);
        path.push_back(path_cell_to_pose(edgeEnd, edgeStart, grid));
        edgeStart = edgeEnd;
    }
    
    // Flip the path to put it in the correct order
    std::reverse(path.begin(), path.end());
    
    return path;
}


cell_t find_next_vertex_in_path(cell_t start, const PredecessorMap& predecessors)
{
    // Follow the parent connections through predecessor map until the deltaX or deltaY changes
    // The goal edge 
    cell_t nextCell = predecessors.at(start);
    int deltaX = start.x - nextCell.x;
    int deltaY = start.y - nextCell.y;
    
    // If both deltaX and deltaY are 0, then we're at the start of the path, so there's nothing to follow
    if((deltaX == 0) && (deltaY == 0))
    {
        return start;
    }
    
    while(true)
    {
        cell_t parentCell = predecessors.at(nextCell);
        
        // As soon as the change between a cell and its parent isn't the same as the initial, then a new line along
        // the path has been found
        if((nextCell.x - parentCell.x != deltaX) || (nextCell.y - parentCell.y != deltaY))
        {
            break;
        }
        
        nextCell = parentCell;
    }
    
    return nextCell;
}


maebot_pose_t path_cell_to_pose(cell_t cell, cell_t previousVertex, const ObstacleDistanceGrid& grid)
{
    auto globalPosition = grid_position_to_global_position(cell, grid);
    
    maebot_pose_t pose;
    pose.x = globalPosition.x;
    pose.y = globalPosition.y;
    pose.theta = std::atan2(previousVertex.y - cell.y, previousVertex.x - cell.x);
    
    return pose;
}

} // namespace eecs467
