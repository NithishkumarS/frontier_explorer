/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Nantha Kumar Sunder, Nithish Sanjeev Kumar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**@file main.cpp
 *
 * @brief Program to explore the unknown path
 *
 * @driver Nithish Sanjeev Kumar
 * @navigator Nantha Kumar Sunder
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#include <ros/ros.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "map.hpp"
#include "explore.hpp"
#include "sensor.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "frontierExplorer");
  ros::NodeHandle nh;
  std::vector<std::vector<int>> path1;
  // Creating a Map object and explore object
  map objMap;
  explore objExplore;
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  ros::Rate loop_rate(10);
  int count = 0;
  int row, col;
  std::vector<int> pos;
  geometry_msgs::Twist rotateBot;
  ros::Publisher rotate = nh.advertise<geometry_msgs::Twist>(
      "mobile_base/commands/velocity", 1);
  // rotating the turtlebot to build the basic map 360DEG
  while (nh.ok() && count < 400) {
    rotateBot.linear.x = 0;
    rotateBot.angular.z = 1.0;
    rotate.publish(rotateBot);
    count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  // stopping the turtlebot from spinning
  rotateBot.linear.x = 0;
  rotateBot.angular.z = 0.0;
  rotate.publish(rotateBot);
  ros::spinOnce();
  loop_rate.sleep();
  int curPosX, curPosY;
  count = 0;
  while (ros::ok()) {
    if (!objMap.requestMap(nh)) {
      ROS_INFO("Map not received");
      exit(-1);
    }
    ROS_INFO("Got map from the node");
    // setting the threshold to get the map
    if (count > 2 && objMap.getResolution() != 0) {
      map_listener.waitForTransform("/map", "/base_link", ros::Time(0),
                                    ros::Duration(3.0));
      map_listener.lookupTransform("/map", "/base_link", ros::Time(0),
                                   map_transform);
      // Obtaining the frontiers
      objMap.frontierSearch(objMap.returnRows(), objMap.returnCols());
      ROS_INFO("Frontier Search completed");
      ros::Duration(5.0).sleep();
      curPosX = abs(
          ((map_transform.getOrigin().x() * objMap.returnCols() / 10))
              + objMap.returnCols() / 2);
      curPosY = abs(
          ((map_transform.getOrigin().y() * objMap.returnRows() / 10))
              + objMap.returnRows() / 2);
      ROS_INFO("Current position: %d %d", curPosX, curPosY);
      // getting the nearest centroid from the list of centroids
      pos = objMap.nearestCentroid(curPosX, curPosY);
      ROS_INFO("End X position: %d", pos[0]);
      ROS_INFO("End Y position: %d", pos[1]);
      ros::Duration(5.0).sleep();
      // getting the Path from pathSearch Function
      objExplore.pathSearch(objMap.getStart(), pos, objMap);
      objExplore.navigate(objMap.getResolution(), objMap, pos);
      int tempCount = 0;
      // Spinning once more to update the map
      while (nh.ok() && tempCount < 400) {
        rotateBot.linear.x = 0;
        rotateBot.angular.z = 1.0;
        rotate.publish(rotateBot);
        tempCount++;
        ros::spinOnce();
        loop_rate.sleep();
      }
      // stopping the turtlebot from spinning
      rotateBot.linear.x = 0;
      rotateBot.angular.z = 0.0;
      rotate.publish(rotateBot);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ros::spinOnce();
    loop_rate.sleep();
    count++;
    // Creating a sensor object
    sensor senObj(objMap);
    // calling the saveRadiationMap to save the image of the radiation
    senObj.saveRadiationMap();
  }
  return 0;
}
