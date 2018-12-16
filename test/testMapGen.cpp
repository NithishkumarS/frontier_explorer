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

/**@file testMapGen.cpp
 *
 * @brief To test the class mapGen
 *
 * @driver Nithish Sanjeev Kumar
 * @navigator Nantha Kumar Sunder
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "map.hpp"
#include "BFS.hpp"

TEST(requestMap, checkNodeActivation) {
// Creating a Node Handle
  ros::NodeHandle nh;
// Bool variable to check the output
  bool output;
// Creating a map object
  map mapObj;
// Calling the requestMap using map object
  output = mapObj.requestMap(nh);
  ros::spinOnce();
// Checking whether the output is true or not
// if not this test case is passed
  EXPECT_EQ(1, output);
}

TEST(frontierSearch, CheckFrontierSearch) {
  // testing for 3x3 occupancy grid
  int height = 3;
  int width = 3;
  //  creating a MAP object
  map mapObj;
  mapObj.frontierSearch(height, width);
  EXPECT_EQ(2, mapObj.getCentroidSize());
}

TEST(checkNeighbour, CheckFrontierSearch) {
  // creating a Map objects
  map mapObj;
  EXPECT_EQ(false, mapObj.checkNeighbour(3, 0, 0));
}
TEST(GetResolution, CheckResolution) {
  // creating a Map objects
  map mapObj;
  EXPECT_EQ(0, mapObj.getResolution());
}
TEST(nearest, checkReturnValue) {
  // Creating the map object
  map mapObj;
  // creating input parameters for nearest function
  int x = 0;
  int y = 0;
  std::queue<std::vector<int>> qList;
  // Creating a temporary vector
  std::vector<int> tempVec;
  // Creating vector to get output from nearest function
  std::vector<int> output;
  // adding the point (2,2) to the queue
  tempVec.push_back(2);
  tempVec.push_back(2);
  qList.push(tempVec);
  tempVec.clear();
  // adding the point (1,1) to the queue
  tempVec.push_back(1);
  tempVec.push_back(1);
  qList.push(tempVec);
  // calling the nearest function and storing it in a queue
  output = mapObj.nearest(qList, x, y);
  // the values should be (1,1) as it is nearer to (0,0) than (2,2)
  EXPECT_EQ(tempVec, output);
}

TEST(computeFrontierCentroid, checkCentroidValue) {
  // Creating a Map Object
  map mapObj;
  // creating a Queue to send as input to computeFrontierCentroid function
  std::queue<std::vector<int>> frontierQueue;
  // Creating a temporary vector
  std::vector<int> tempVec, compareVec;
  // adding the point (2,2) to the queue
  tempVec.push_back(2);
  tempVec.push_back(2);
  frontierQueue.push(tempVec);
  tempVec.clear();
  // adding the point (0,0) to the queue
  tempVec.push_back(0);
  tempVec.push_back(0);
  frontierQueue.push(tempVec);
  tempVec.clear();
  // pushing the (1,1) to compare Vector
  compareVec.push_back(0);
  compareVec.push_back(0);
  // Calling the function computeFrontierCentroid
  tempVec = mapObj.computeFrontierCentroid(frontierQueue);
  EXPECT_EQ(compareVec, tempVec);
}
TEST(ReturnRows, checkReturnValueOfRow) {
  map mapObj;
  EXPECT_EQ(3, mapObj.returnRows());
}
TEST(ReturnCols, checkReturnValueOfCols) {
  map mapObj;
  EXPECT_EQ(3, mapObj.returnCols());
}
TEST(GetGridValue, checkGetGridValue) {
  map mapObj;
  EXPECT_EQ(0, mapObj.getGridValue(0, 0));
}
/*
 void mapcallback(const nav_msgs::OccupancyGrid& msg);
 std::vector<int> getStart();
 */
