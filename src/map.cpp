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
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCURccenEMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**@file map.cpp
 *
 * @brief To implementation of the class which implements the map
 *
 * @driver Nithish Sanjeev Kumar
 * @navigator Nantha Kumar Sunder
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <queue>
#include <cmath>
#include "map.hpp"
#include "../include/BFS.hpp"

// Contructor initializes the grid
map::map() {
  rows = 3;
  cols = 3;
  mapResolution = 0;

  grid.resize(3);
  for (auto& i : grid) {
    i.resize(cols);
  }
  int a = 0;
  std::vector<int> values = { 0, 0, 0, 0, 1, 1, -1, -1, 1 };
  for (auto& i : grid) {
    for (auto& j : i) {
      j = values[a];
      a++;
    }
  }
}
// Function to run the /map topic subscriber
bool map::requestMap(ros::NodeHandle &nh) {
  OGM_subscriber = nh.subscribe("map", 10, &map::mapcallback, this);
  // ROS_INFO("checkpoint12");
  return true;
}
// Call back function
void map::mapcallback(const nav_msgs::OccupancyGrid& msg) {
  int startX = static_cast<int>(msg.info.origin.position.x);
  int startY = static_cast<int>(msg.info.origin.position.y);
  start.push_back(0 - startY);
  start.push_back(0 - startX);
  ROS_INFO("In callback");
  ROS_INFO("Received a %d X %d map @ %.3f m/px \n", msg.info.width,
           msg.info.height, msg.info.resolution);
  rows = msg.info.height;
  cols = msg.info.width;
  mapResolution = msg.info.resolution;

  grid.resize(rows);
  for (auto& i : grid) {
    i.resize(cols);
  }
  // int currCell = 0;
  int xCount = 0;
  int yCount = 0;
  for (auto& i : grid) {
    yCount = 0;
    for (auto& j : i) {
      j = static_cast<int> (msg.data[yCount + xCount * msg.info.width]);
      if (j > 1) {
        j = -1;
      }
      yCount = yCount + 1;
    }
    xCount = xCount + 1;
  }
}

int map::getGridValue(int x, int y) {
  return grid[x][y];
}
// Function to return rows
int map::returnRows() {
  return rows;
}
// Function to return cols
int map::returnCols() {
  return cols;
}
// Function to return centroid size
int map::getCentroidSize() {
  return centroidQueue.size();
}
// Function to check if there are free neighbours
bool map::checkNeighbour(int xPoint, int yPoint, int mode) {
  std::vector<std::vector<int> > neighbourPoints;
  std::vector<int> tempVec;
  int move[3] = { 1, -1, 0 };
  for (auto & x : move) {
    for (auto & y : move) {
      if (!(y == 0 && x == 0) && ((yPoint + y) >= 0 && (xPoint + x) >= 0)
          && ((yPoint + y) < grid[0].size()) && ((xPoint + x) < grid.size())) {
        if (grid[xPoint + x][yPoint + y] == mode) {
          tempVec.push_back(yPoint + y);
          tempVec.push_back(xPoint + x);
          neighbourPoints.push_back(tempVec);
          tempVec.clear();
        }
      }
    }
  }
  if (neighbourPoints.empty()) {
    return false;
  } else {
    return true;
  }
}
// Function to return point nearest to centroid
std::vector<int> map::nearest(const std::queue<std::vector<int>>& q,
                              int centroidx, int centroidy) {
  std::queue<std::vector<int>> tempQ;
  tempQ = q;
  // return tempQ.front();
  int small = 0;
  int dist = 0;
  std::vector<int> centroid, v;
  int flag = 1;
  while (!tempQ.empty()) {
    v = (tempQ.front());
    dist = sqrt((v[0] - centroidx) ^ 2 + (v[1] - centroidy) ^ 2);
    if (flag == 1) {
      small = dist;
      flag++;
    }
    if (small >= dist) {
      centroid = tempQ.front();
      small = dist;
    }
    tempQ.pop();
  }
  return centroid;
}
// Function to compute centroid
std::vector<int> map::computeFrontierCentroid(
    const std::queue<std::vector<int>>& q) {
  std::vector<int> v, output;
  std::queue<std::vector<int>> tempQ;
  tempQ = q;
  int centroidx = 0, centroidy = 0;
  int count = 0;
  int xpoints = 0, ypoints = 0;
  while (!tempQ.empty()) {
    v = (tempQ.front());
    tempQ.pop();
    xpoints = xpoints + v[0];
    ypoints = ypoints + v[1];
    count++;
  }
  centroidx = xpoints / count;
  centroidy = ypoints / count;

  std::vector<int> centroid = nearest(q, centroidx, centroidy);
  return centroid;
}

void map::frontierSearch(int height, int width) {
  BFS bfsObj(height, width);
  std::vector<int> tempVec;
  std::vector<std::vector<int>> map;
  std::queue<std::vector<int>> tempQueue, tempFrontierQueue;
  std::vector<std::vector<int>> vec(height, std::vector<int>(width, 1));
  int xPoint, yPoint;
  while (!frontierQueue.empty()) {
    frontierQueue.pop();
  }
  while (!centroidQueue.empty()) {
    centroidQueue.pop();
  }
  map = grid;
  xPoint = 0;
  yPoint = 0;
  int isExplored = 0;
  tempQueue = explored;
  for (auto & i : map) {
    yPoint = 0;
    for (auto & j : i) {
      tempVec.push_back(yPoint);
      tempVec.push_back(xPoint);
      while (!tempQueue.empty()) {
        if (tempQueue.front() == tempVec) {
          isExplored = 1;
        }
        tempQueue.pop();
      }
      if (j == -1 && checkNeighbour(xPoint, yPoint, 0)) {
        frontierQueue.push(tempVec);
        explored.push(tempVec);
      }
      tempVec.clear();
      isExplored = 0;
      yPoint = yPoint + 1;
    }
    xPoint = xPoint + 1;
  }
  xPoint = 0;
  yPoint = 0;
  tempQueue = frontierQueue;
  while (!tempQueue.empty()) {
    tempVec = tempQueue.front();
    tempQueue.pop();
    tempFrontierQueue = bfsObj.computeBFS(frontierQueue, tempVec);
    centroidQueue.push(computeFrontierCentroid(tempFrontierQueue));
    tempVec.pop_back();
    while (!tempFrontierQueue.empty()) {
      tempFrontierQueue.pop();
    }
  }
}
std::vector<int> map::nearestCentroid(int startX, int startY) {
  std::queue<std::vector<int>> tempQ;
  tempQ = centroidQueue;
  float small = 0;
  float dist = 0;
  std::vector<int> centroid, v;
  v = centroidQueue.front();
  int flag = 1;
  while (!tempQ.empty()) {
    v = tempQ.front();
    dist = sqrt((v[0] - startX) ^ 2 + (v[1] - startY) ^ 2);
    if (flag == 1) {
      small = dist;
      flag++;
    }
    if (small >= dist) {
      centroid = tempQ.front();
      small = dist;
    }
    tempQ.pop();
    v.clear();
  }
  // Removing the first element
  centroidQueue.pop();
  return centroid;
}

// Function to return origin vector
std::vector<int> map::getStart() {
  return start;
}

float map::getResolution() {
  return mapResolution;
}

map::~map() {
}
