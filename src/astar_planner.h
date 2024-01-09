#include <string.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <set>

//for global path planner
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;
using std::string;

#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

//Struct represents a grid and its fcost
struct GridSquare
{
    int currentGridSquare;
    float fCost;
};

namespace astar_planner_plugin
{
    class AStarPlanner : public nav_core::BaseGlobalPlanner
    {
        public:
        ros::NodeHandle RosNodeHandle;
        float originX;
        float originY;
        float resolution;
        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;
        bool initialized_;
        int width;
        int height;
        AStarPlanner(ros::NodeHandle &);
        AStarPlanner();
        AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        //overriden methods from interface nav_core::BaseglobalPlanner
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

        //helper methods
        void convertToMapCoordinates(float &x, float &y);
        int getGridSquareIndex(float x, float y);
        void getGridSquareCoordinates(int index, float &x, float &y);
        bool isCoordinateInBounds(float x, float y);
        vector<int> runAstarOnGrid(int startGridSquare, int goalGridSquare);
        vector<int> findPath(int startGridSquare, int goalGridSquare, float gScore[]);
        vector<int> constructPath(int startGridSquare, int goalGridSquare, float gScore[]);
        void addNeighborGridSquareToOpenList(multiset<GridSquare> &OPL, int neighborGridSquare, int goalGridSquare, float gScore[]);
        vector<int> findFreeNeighborGridSquare(int currentSquare);
        bool isStartAndGoalValid(int startGridSquare, int goalGridSquare);
        float getMoveCost(int gridSquareInd1, int gridSquareInd2);
        float getMoveCost(int i1, int j1, int i2, int j2);
        float calculateHScore(int gridSquareInd, int goalGridSquare);
        int calculateGridSquareInd(int i, int j);
        int getGridSquareRowInd(int index);
        int getGridSquareColInd(int index);
        bool isFree(int gridSquareInd);
        bool isFree(int i, int j);
    };
};

#endif