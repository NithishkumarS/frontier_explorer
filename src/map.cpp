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
map::~map() {}
