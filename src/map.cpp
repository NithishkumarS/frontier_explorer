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

map::~map() {}
