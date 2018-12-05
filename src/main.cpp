#include <ros/ros.h>
#include "map.hpp"

int main(int argc, char** argv ) {

  ros::init(argc, argv, "Get_Map_node");
  ros::NodeHandle nh;
  map obj;
  ros::Rate loop_rate(10);
  int count =0;
  while(ros::ok() && count < 3){
      if(!obj.requestMap(nh)){
        ROS_INFO("Checkpoint1");
        exit(-1);
      }
  ROS_INFO("Checkpoint2");
  obj.getOccupancyGrid();

  ROS_INFO("Checkpoint3");
  ros::spinOnce();
  
  loop_rate.sleep();
  count++;
  }
  return 0;
}
