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

/**@file explore.cpp
 *
 * @brief To implement the functions of the class explore
 *
 * @driver Nantha Kumar Sunder
 * @navigator Nithish Sanjeev Kumar
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cstdlib>
#include <algorithm>
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/Twist.h"
#include "explore.hpp"

explore::explore() {
}

void explore::findObstacleFreeNeighbours(std::vector<int> v,
                                         std::queue<std::vector<int> >& list,
                                         map& obj) {
  // setting the ]xPoint and ypoint
  int xPoint = v[0];
  int yPoint = v[1];
  std::vector<int> tempVec;
  // list of all possible moves
  int move[3] = { 1, -1, 0 };

  for (auto & x : move) {
    for (auto & y : move) {
      // Condition to check if the subscript of the matrix is not out of bound
      if (!(y == 0 && x == 0) && ((yPoint + y) >= 0 && (xPoint + x) >= 0)
          && ((yPoint + y) < obj.returnCols() - 1)
          && ((xPoint + x) < obj.returnRows() - 1)) {
        // Checking whether the grid value is occupied or not
        if (obj.getGridValue(xPoint + x, yPoint + y) != 1) {
          // Pushing the point to list of obstacle less neighbour
          tempVec.push_back(yPoint + y);
          tempVec.push_back(xPoint + x);
          list.push(tempVec);
          tempVec.clear();
        }
      }
    }
  }
}

bool explore::computeShortestPath(std::vector<int> start, std::vector<int> End,
                                  std::vector<std::vector<int> >& path1,
                                  map& obj) {
  //
  int goalFound = 0;
  path.clear();
  std::vector<std::vector<std::vector<int> > > parent;
  std::queue<std::vector<int> > pQueue, list;
  std::vector<std::vector<int> > explored;
  std::vector<int> v, i, temp;
  ROS_INFO("Start Point X %d", start[0]);
  ROS_INFO("Start point Y%d", start[1]);
  ROS_INFO("End Point X %d", End[0]);
  ROS_INFO("End point Y%d", End[1]);
  // taking the rows and columns of the map
  int rows = obj.returnRows();
  int cols = obj.returnCols();
  parent.resize(rows);
  // initializing the parent node as 0
  for (auto& i : parent) {
    i.resize(cols);
    for (auto& j : i) {
      j.resize(2);
      for (auto& k : j)
        k = 0;
    }
  }
  // Initializing the explored node as 0
  explored.resize(rows);
  for (auto& i : explored) {
    i.resize(cols);
    for (auto& j : i)
      j = 0;
  }
  // Find the path using the BFS
  pQueue.push(start);
  while (!pQueue.empty()) {
    v = pQueue.front();
    pQueue.pop();
    findObstacleFreeNeighbours(v, list, obj);
    while (!list.empty()) {
      i = list.front();
      list.pop();
      // checking if the current point is already visited or not
      if (explored[i[0]][i[1]] == 0) {
        explored[i[0]][i[1]] = 1;
        parent[i[0]][i[1]] = v;
        pQueue.push(i);
        if (End[0] == i[0] && End[1] == i[1]) {
          goalFound = 1;
        }
      }
    }
  }
  // If the goal is found
  if (goalFound == 1) {
    path1.push_back(End);
    temp = End;
    while (temp != start) {
      // adding the parent node
      temp = parent[temp[0]][temp[1]];
      path1.push_back(temp);
    }
    goalFound = 0;
    path = path1;
    ROS_INFO("out of computePathSearch");
    return true;
  }
  ROS_INFO("out of computePathSearch");
  return false;
}

void explore::pathSearch(std::vector<int> start, std::vector<int> end,
                         map& obj) {
  bool check;
  ROS_INFO("Inside path Search");
  // calling the compute path search
  check = computeShortestPath(start, end, path, obj);
  ROS_INFO("Path found %d", check);
  ROS_INFO("out of path search");
}

bool explore::navigate(float resolution, map& obj, std::vector<int> pos) {
  ROS_INFO("Inside Navigation");
  ros::Duration(5.0).sleep();
  bool targetReached = false;
  std::vector<int> tempVec;
  float goalX, goalY;
  ros::Rate loop_rate(10);
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  map_listener.waitForTransform("/map", "/base_link", ros::Time(0),
                                ros::Duration(3.0));
  map_listener.lookupTransform("/map", "/base_link", ros::Time(0),
                               map_transform);
  std::reverse(path.begin(), path.end());
  int size;
  size = path.size();
  tempVec = path[path.size() - 1];
  // Initializing the move base goal
  move_base_msgs::MoveBaseGoal goal;
  // Choosing the frame id to post goal message as map
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  // Initializing the simple action client to enable sending goal parameters
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> goalState(
      "move_base", true);
  // Waiting for the server to switch on
  while (!goalState.waitForServer(ros::Duration(10.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  tempVec = pos;
  goalX = (static_cast<float>(tempVec[0]) - obj.returnCols() / 2)
      / obj.returnCols() * 10;
  goalY = (static_cast<float>(tempVec[1]) - obj.returnRows() / 2)
      / obj.returnRows() * 10;

    ROS_INFO("tempVec[0] %f %f", goalX, goalY);
  // Setting goal points
  goal.target_pose.pose.position.x = goalX;
  goal.target_pose.pose.position.y = goalY;
  // setting the orientation
  goal.target_pose.pose.orientation.w = 1.0;
  while (!goalState.waitForServer(ros::Duration(10.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("Initiating the server");
  // sending the goal to move base
  goalState.sendGoal(goal);
  goalState.waitForResult(ros::Duration(20.0));

  int count = 0;
  if (goalState.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Goal is Reached");
    // Rotating the Bot to update the occupancy grid
    // Rotating the Bot 180 deg
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(
        3.14);
    goal.target_pose.pose.orientation = orientation;
    goalState.sendGoal(goal);
    goalState.waitForResult(ros::Duration(10.0));
    // Rotating the Bot 180 deg again
    orientation = tf::createQuaternionMsgFromYaw(3.14);
    goal.target_pose.pose.orientation = orientation;
    goalState.sendGoal(goal);
    goalState.waitForResult(ros::Duration(10.0));
    // Setting the return quantity true
    targetReached = true;
  } else {
    ROS_WARN("Goal is not reached");
    // going to the origin of the map
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    // setting the orientation
    goal.target_pose.pose.orientation.w = 1.0;
    while (!goalState.waitForServer(ros::Duration(10.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Initiating the server");
    // sending the goal to move base
    goalState.sendGoal(goal);
    goalState.waitForResult(ros::Duration(20.0));
  }
  return targetReached;
  ROS_INFO("out of navigation");
}

explore::~explore() {
}
