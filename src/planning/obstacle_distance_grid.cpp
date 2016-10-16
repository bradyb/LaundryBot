#include <planning/obstacle_distance_grid.hpp>
#include <mapping/occupancy_grid.hpp>
#include <queue>

namespace eecs467
{
    
typedef Point<int> cell_t;


struct DistanceNode
{
    cell_t obstacle;        // Location of the nearest obstacle to the node
    cell_t cell;            // Cell represented by this node
    double distance;        // Distance to the node -- in cells
    
    bool operator<(const DistanceNode& rhs) const
    {
        return distance < rhs.distance;
    }
    
    DistanceNode(cell_t cell, cell_t obstacle)
    : obstacle(obstacle)
    , cell(cell)
    , distance(distance_between_points(cell, obstacle))
    {
    }
};


void enqueue_obstacle_cells(const OccupancyGrid& map, std::priority_queue<DistanceNode>& searchQueue);
void expand_node(const DistanceNode& node, 
                 const ObstacleDistanceGrid& grid, 
                 std::priority_queue<DistanceNode>& searchQueue);

bool is_cell_free_space(cell_t cell, const OccupancyGrid& map);
bool is_cell_occupied(cell_t cell, const OccupancyGrid& map);

    
    
ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    initializeDistances(map);
    
    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(map, searchQueue);
    
    while(!searchQueue.empty())
    {
        DistanceNode nextNode = searchQueue.top();
        searchQueue.pop();
        
        if((nextNode.distance < distance(nextNode.cell.x, nextNode.cell.y)) 
            || (nextNode.distance == 0.0f))
        {
            distance(nextNode.cell.x, nextNode.cell.y) = nextNode.distance;
            expand_node(nextNode, *this, searchQueue);
        }
    }
    
    // After the distances are found, convert them from cell distances to metric distances
    std::transform(cells_.begin(), cells_.end(), cells_.begin(), [&map](double distInCells) { 
        return distInCells * map.metersPerCell();
    });
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < static_cast<int>(width_)) && (y >= 0) && (y < static_cast<int>(height_));
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == static_cast<int>(map.widthInCells())) && (height_ == static_cast<int>(map.heightInCells())))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}


void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map)
{
    // Set all distances to 0 -- unknown and obstacles will stay 0.
    std::fill(cells_.begin(), cells_.end(), 0.0f);
    
    // Anything that is free space, set to a very large number, so the search can easily expand to anything with
    // a shorter distance than the stored distance. Avoids the need to keep a visited list.
    int width  = map.widthInCells();
    int height = map.heightInCells();
    
    cell_t cell;
    
    for(cell.y = 0; cell.y < height; ++cell.y)
    {
        for(cell.x = 0; cell.x < width; ++cell.x)
        {
            if(is_cell_free_space(cell, map))
            {
                distance(cell.x, cell.y) = std::numeric_limits<float>::max();
            }
        }
    }
}


void enqueue_obstacle_cells(const OccupancyGrid& map, std::priority_queue<DistanceNode>& searchQueue)
{
    int width  = map.widthInCells();
    int height = map.heightInCells();
    
    cell_t cell;
    
    for(cell.y = 0; cell.y < height; ++cell.y)
    {
        for(cell.x = 0; cell.x < width; ++cell.x)
        {
            if(is_cell_occupied(cell, map))
            {
                // Occupied cells are their own closest obstacle
                searchQueue.push(DistanceNode(cell, cell));
            }
        }
    }
}


void expand_node(const DistanceNode& node, 
                 const ObstacleDistanceGrid& grid, 
                 std::priority_queue<DistanceNode>& searchQueue)
{
    // Perform a four-way expansion of each cell
    const int xDeltas[4] = { 1, -1, 0, 0 };
    const int yDeltas[4] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < 4; ++n)
    {
        cell_t adjacentCell(node.cell.x + xDeltas[n], node.cell.y + yDeltas[n]);
        
        // A cell will be enqueued if:
        // it is in the grid...
        if(grid.isCellInGrid(adjacentCell.x, adjacentCell.y))
        {
            DistanceNode adjacentNode(adjacentCell, node.obstacle);
            
            // ...and closer than any other obstacle seen so far
            if(adjacentNode.distance < grid(adjacentCell.x, adjacentCell.y))
            {
                searchQueue.push(adjacentNode);
            }
        }
    }
}


bool is_cell_free_space(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) < 0;
}

bool is_cell_occupied(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) > 0;
}

} // namespace eecs467
