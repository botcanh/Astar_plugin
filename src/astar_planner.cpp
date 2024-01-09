#include "astar_planner.h"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

//BaseGlobalPlanner Plugin
PLUGINLIB_EXPORT_CLASS(astar_planner_plugin::AStarPlanner, nav_core::BaseGlobalPlanner);

int value;
int mapSize;
bool *occupancyGridMap;

float infinity = std::numeric_limits<float>::infinity();

float tBreak;

namespace astar_planner_plugin
{
    //Default constructor
    AStarPlanner::AStarPlanner(ros::NodeHandle &nh)
    {
        RosNodeHandle = nh;
    }

    //Constructors that initialize costmap and params
    AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    //Costmap2DROS is a class
    void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if(!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            mapSize = width * height;
            tBreak = 1 + 1 / (mapSize);
            value = 0;

            /*----- Define the closed list -----*/
            occupancyGridMap = new bool[mapSize];   //This is the closed List

            for(unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++) //every row
            {
                for(unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++) // every col
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                    if (cost == 0)
                    {
                        occupancyGridMap[iy * width + ix] = true;
                    }else
                    {
                        occupancyGridMap[iy * width + ix] = false;
                    }
                }
            }
            ROS_INFO("AStar planner has been initialized!!");
            initialized_ = true;
        }
        else 
        ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("The planner hasnt been initialized, please call initialize()");
            return false;
        }
        ROS_DEBUG("Got a start: %.2f, %.2f and a goal: %.2f, %.2f",start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
        plan.clear();

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("Wrong frame id. goal frame id was %s, but the global frame id was %s.", goal.header.frame_id.c_str(), costmap_ros_->getBaseFrameID().c_str());
            return false;
        }

        //convert start- goal position
        float startX = start.pose.position.x;
        float startY = start.pose.position.y;
        float goalX = goal.pose.position.x;
        float goalY = goal.pose.position.y;

        //convert to map coordinates RELATIVES TO COST MAP ORIGIN
        convertToMapCoordinates(startX, startY);
        convertToMapCoordinates(goalX, goalY);

        int startGridSquare;
        int goalGridSquare;

        if(isCoordinateInBounds(startX, startY) && isCoordinateInBounds(goalX, goalY))
        {
            startGridSquare = getGridSquareIndex(startX, startY);
            goalGridSquare = getGridSquareIndex(goalX, goalY);
        } else
        {
            ROS_WARN("start or goal position is out of the map");
            return false;
        }

        //cal the global planner
        if(isStartAndGoalValid(startGridSquare, goalGridSquare))
        {
            vector<int> bestPath;
            bestPath.clear();

            bestPath = runAstarOnGrid(startGridSquare, goalGridSquare);

            //if the global planner find a path- trace path from GOAL TO START
            if(bestPath.size() > 0)
            {
                //convert the path
                for(int i = 0; i < bestPath.size(); i++)
                {
                    float x = 0.0;
                    float y = 0.0;

                    float previous_x = 0.0;
                    float previous_y = 0.0;

                    int index = bestPath[i];
                    int previous_index;
                    getGridSquareCoordinates(index, x, y);

                    if(i != 0)
                    {
                        previous_index = bestPath[i - 1];
                    }else
                    {
                        previous_index = index; // the start goal
                    }

                    getGridSquareCoordinates(previous_index, previous_x, previous_y);

                    //Orient the robot towards target
                    tf::Vector3 vectorToTarget;
                    vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
                    float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

                    geometry_msgs::PoseStamped pose = goal;

                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

                    plan.push_back(pose);
                }
                return true;
            }
            else
            {
                ROS_WARN("The planner failed to find a path");
                return false;
            }
        }
        else
        {
            ROS_WARN("Not valid start or goal");
            return false;
        }
    }

    //Function to adjust start and goal relative to origin point on map
    void AStarPlanner::convertToMapCoordinates(float &x, float &y)
    {
        x = x - originX;
        y = y - originY;
    }

    //Function to get index of grid square on map - given square coordinates
    int AStarPlanner::getGridSquareIndex(float x, float y)
    {
        int gridSquare;

        float newX = x / resolution;
        float newY = y / resolution;

        gridSquare = calculateGridSquareInd(newX, newY);

        return gridSquare;
    }

    //Function to get gridSquare coordinates given index - input: x,y that has been changed to be relatived to origin
    void AStarPlanner::getGridSquareCoordinates(int index, float &x, float &y)
    {
        x = getGridSquareColInd(index) * resolution;
        y = getGridSquareRowInd(index) * resolution;

        x = x + originX;
        y = y + originY;
    }

    bool AStarPlanner::isCoordinateInBounds(float x, float y)
    {
        bool valid = true;

        if(x > (width * resolution) || y > (height * resolution))
        {
            valid = false;
        }

        return valid;
    }

    //Run AStar on grid
    vector<int>AStarPlanner::runAstarOnGrid(int startGridSquare, int goalGridSquare)
    {
        vector<int> bestPath;
        float g_Score[mapSize];
        
        for(uint i = 0; i < mapSize; i++)
        {
            g_Score[i] = infinity;
        }

        bestPath = findPath(startGridSquare, goalGridSquare, g_Score);

        return bestPath;
    }

    vector<int> AStarPlanner::findPath(int startGridSquare, int goalGridSquare, float g_score[])
    {
        value++;
        vector<int> bestPath;
        vector<int> emptyPath;
        GridSquare gridSq;

        multiset<GridSquare> openSquareList;
        int currentGridSquare;

        //calculate g_score and f_score of the start position
        g_score[startGridSquare] = 0;
        gridSq.currentGridSquare = startGridSquare;
        gridSq.fCost = g_score[startGridSquare] + calculateHScore(startGridSquare, goalGridSquare);

        //add the start gridSquare to the openlist
        openSquareList.insert(gridSq);
        currentGridSquare = startGridSquare;

        //while open list is not empty - till goal square is reached 
        while(!openSquareList.empty() && g_score[goalGridSquare] == infinity)
        {
            //choose the grid square that has the lowest fCost in the openList
            currentGridSquare = openSquareList.begin()->currentGridSquare;
            //remove it from the openList
            openSquareList.erase(openSquareList.begin());
            //search all the neighbors of the current square
            vector<int> neighborGridSquare;
            neighborGridSquare = findFreeNeighborGridSquare(currentGridSquare);
            for(uint i = 0; i < neighborGridSquare.size(); i ++)
            {
                //if the neighbor's g_score == INF --> unvisited square
                if(g_score[neighborGridSquare[i]] == infinity)
                {
                    g_score[neighborGridSquare[i]] = g_score[currentGridSquare] + getMoveCost(currentGridSquare, neighborGridSquare[i]);
                    addNeighborGridSquareToOpenList(openSquareList, neighborGridSquare[i], goalGridSquare, g_score);
                }
            }
        }

        if(g_score[goalGridSquare] != infinity) //the goal has no been reached
        {
            bestPath = constructPath(startGridSquare, goalGridSquare, g_score);
            return bestPath;
        }else
        {
            ROS_INFO("Fail to find a path");
            return emptyPath;
        }
    }

    //function that constructs the path we got from findPath, return the vector of the gridSquare that lie on path.
    vector<int>AStarPlanner::constructPath(int startGridSquare, int goalGridSquare, float g_score[])
    {
        vector<int> bestPath;
        vector<int> path;

        path.insert(path.begin() + bestPath.size(), goalGridSquare);
        int currentGridsquare = goalGridSquare;

        //search from goal
        while(currentGridsquare != startGridSquare)
        {
            vector<int> neighborGridSquares;
            neighborGridSquares = findFreeNeighborGridSquare(currentGridsquare);

            vector<float> gScoresNeighbors;
            for(uint i = 0; i < neighborGridSquares.size(); i++)
            {
                gScoresNeighbors.push_back(g_score[neighborGridSquares[i]]);
            }
            //min_element get the lowest gScore
            //distance helps find the index of the lowest gScore node
            int posMinScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
            currentGridsquare = neighborGridSquares[posMinScore];

            //insert the neighbor to the path
            path.insert(path.begin() + bestPath.size(), currentGridsquare);
        }
        for(uint i = 0; i < path.size(); i++)
        {
            //path is store from goal -> start.But insert bestPath from start -> goal
            bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);
        }
        return bestPath;
    }

    //function that helps add unexplored neighbours to the openList.
    void AStarPlanner::addNeighborGridSquareToOpenList(multiset<GridSquare> &OPL, int neighborGridSquare, int goalGridSquare, float g_score[])
    {
        GridSquare gridSq;
        gridSq.currentGridSquare = neighborGridSquare;
        gridSq.fCost = g_score[neighborGridSquare] + calculateHScore(neighborGridSquare, goalGridSquare);
        OPL.insert(gridSq);
    }

    vector<int> AStarPlanner::findFreeNeighborGridSquare(int gridSquare)
    {
        int rowIndex = getGridSquareRowInd(gridSquare);
        int colIndex = getGridSquareColInd(gridSquare);
        int neighborIndex;
        vector<int>freeNeighBorGridSquare;

        for(int i = -1; i <= 1; i ++)
        {
            for(int j = -1; j <= 1; j ++)
            {
                //check whether the index is valid
                if ((rowIndex + i >= 0) && (rowIndex + i < height) && (colIndex + j >= 0) && (colIndex + j < width) && (!(i == 0 && j == 0)))
                {
                    neighborIndex = ((rowIndex + i) * width) + (colIndex + j);

                    if (isFree(neighborIndex))
                    freeNeighBorGridSquare.push_back(neighborIndex);
                }
            }
        }
        return freeNeighBorGridSquare;
    }

    //check if start and goal grid are valid and reachable
    bool AStarPlanner::isStartAndGoalValid(int startGridSquare, int goalGridSquare)
    {
        bool isValid = true;
        bool isFreeStartGridSquare = isFree(startGridSquare);
        bool isFreeGoalGridSquare = isFree(goalGridSquare);
        
        if(startGridSquare == goalGridSquare)
        {
            isValid = false;
        }else
        {
            if(!isFreeStartGridSquare && !isFreeGoalGridSquare)
            {
                isValid = false;
            }else
            {
                if(!isFreeStartGridSquare)
                {
                    isValid = false;
                }else{
                    if(findFreeNeighborGridSquare(goalGridSquare).size() == 0)
                    {
                        isValid = false;
                    }else
                    {
                        if(findFreeNeighborGridSquare(startGridSquare).size() == 0)
                        {
                            isValid = false;
                        }
                    }
                }
            }
        }
        return isValid;
    }
    //get move cost to move to neighbor
    float AStarPlanner::getMoveCost(int i1, int j1, int i2, int j2)
    {
        float moveCost = infinity;
        if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
        {

            moveCost = 1.4;
        }
        //if gridSquare(i2,j2) exists in the horizontal or vertical line with gridSquare(i1,j1)
        else
        {
            if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
            {

            moveCost = 1;
            }
        }
        return moveCost;
    }
    float AStarPlanner::getMoveCost(int gridSquareIndex1, int gridSquareIndex2)
    {
        int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

        i1 = getGridSquareRowInd(gridSquareIndex1);
        j1 = getGridSquareColInd(gridSquareIndex1);
        i2 = getGridSquareRowInd(gridSquareIndex2);
        j2 = getGridSquareColInd(gridSquareIndex2);

        return getMoveCost(i1, j1, i2, j2);
    }

    float AStarPlanner::calculateHScore(int startGridSquare, int goalGridSquare)
    {
        int x1 = getGridSquareRowInd(goalGridSquare);
        int y1 = getGridSquareColInd(goalGridSquare);
        int x2 = getGridSquareRowInd(startGridSquare);
        int y2 = getGridSquareColInd(startGridSquare);
        return abs(x1 - x2) + abs(y1 - y2);
    }

    int AStarPlanner::calculateGridSquareInd(int i, int j)
    {
        return (i * width) + j;
    }
    //get row ind
    int AStarPlanner::getGridSquareRowInd(int index)
    {
        return index / width;
    }
    //get col ind
    int AStarPlanner::getGridSquareColInd(int index)
    {
        return index % width;
    }

    //check if a gridSquare is free
    bool AStarPlanner::isFree(int gridSquareIndex)
    {
    return occupancyGridMap[gridSquareIndex];
    }  
};

/**
  Overloaded operator for comparing cost among two gridSquares.

**/
bool operator<(GridSquare const &c1, GridSquare const &c2) { return c1.fCost < c2.fCost; }
