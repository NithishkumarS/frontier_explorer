#include <ros/ros.h>
#include <vector>
#include <cmath>
#include "map.hpp"

map::map(){
rows = 0;
cols =0;
mapResolution =0;
}

bool map::requestMap(ros::NodeHandle &nh) {
  OGM_subscriber = nh.subscribe("map", 10, &map::mapcallback, this);
  ROS_INFO("checkpoint12");
  return true;
}

void map::mapcallback(const nav_msgs::OccupancyGrid& msg) {
  ROS_INFO("In callback");
  ROS_INFO("Received a %d X %d map @ %.3f m/px \n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution );
  rows = msg.info.height;
  cols = msg.info.width;
  ROS_INFO("rows: %d \n Cols :%d",rows,cols);
  mapResolution = msg.info.resolution;
  ROS_INFO("Resolution: %f ", mapResolution );
  // Dynamically resize the grid
  grid.resize(rows);
  for(auto& i:grid) { //int i = 0; i < rows; i++) {
   i.resize(cols);
  }
  int currCell = 0;
  for(auto& i:grid) {//int i = 0; i < rows; i++) {
     for(auto& j:i) { //int j = 0; j < cols; j++) {
        if(msg.data[currCell] == 0) // unoccupied cell
          j = 0;
        else if (msg.data[currCell] == -1){
          j = -1;
        }
        else
          j = 1 ;
       //ROS_INFO("%d", j);
        currCell++;
     }

  }
  ROS_INFO("Grid size:%d", grid.size());
  ROS_INFO("checkpoint- mapcallback");
}

void map::getOccupancyGrid() {
 ROS_INFO("Checkpoint4");
 ROS_INFO("%d",grid.size());
 //std::cout << "The map";
 //ROS_INFO("%d",grid[0][0]);
 for(auto& i:grid) {
   for( auto& j:i){
     ROS_INFO("Checkpoint5");
     ROS_INFO("%d", j);
   }
 ROS_INFO("\n");
 }

}

int map::getGridValue(int x, int y) {
  return grid[x][y];
}

int map::returnRows(){
  return rows;
}
int map::returnCols() {
  return cols;
}
int map::getCentroidSize(){
  return centroidQueue.size();
}

bool map::checkNeighbour(int xPoint, int yPoint, int mode) {
  std::vector< std::vector <int> > neighbourPoints;
  std::vector<int> tempVec;
  int move[3] = {1, -1 ,0};
  for (auto & x : move) {
      for (auto & y : move) {
      if ( y!=0 && x!=0 ){
        if(grid[xPoint+x][yPoint+y] == mode) {
          tempVec.push_back(yPoint+y);
          tempVec.push_back(xPoint+x);
          neighbourPoints.push_back(tempVec);
          tempVec.clear();
        }
      }
      }
  }
  if (neighbourPoints.empty()){
    return false;
  } else {
    return true;
  }
}

map::~map() {}
