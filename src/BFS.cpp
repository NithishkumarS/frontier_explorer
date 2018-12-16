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
 * @brief To implement the function of the class BFS
 *
 * @driver Nantha Kumar Sunder
 * @navigator Nithish Sanjeev Kumar
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#include <ros/ros.h>
#include <queue>
#include <vector>
#include "BFS.hpp"

BFS::BFS(int rows, int cols) {
  // Resize the explored
  explored.resize(rows);
  // Resize the each element of the explored
  for (auto& i : explored) {
    i.resize(cols);
    for (auto& j : i)
      j = 0;
  }
  // Resize the distance
  distance.resize(rows);
  // Resixe the each element of the distance
  for (auto& i : distance) {
    i.resize(cols);
    for (auto& j : i)
      j = 0;
  }
}

int BFS::getExplored(int x, int y) {
  // Return the value of explored at (x,y)
  return explored[x][y];
}

int BFS::getDistance(int x, int y) {
  // Return the value of distance at (x,y)
  return distance[x][y];
}

void BFS::neighbourList(const std::vector<std::vector<int>>& frontierVector,
                        std::vector<int> v,
                        std::queue<std::vector<int> >& list) {
  // Intializing the x and y point
  int xPoint = v[0];
  int yPoint = v[1];
  std::vector<int> tempVec;
  // list of all possible moves along any direction
  int move[3] = { 1, -1, 0 };
  // finding the neighbouring grid
  for (auto & x : move) {
    for (auto & y : move) {
      tempVec.push_back(yPoint + y);
      tempVec.push_back(xPoint + x);
      for (auto & k : frontierVector) {
        if (k == tempVec) {
          // pushing the neighbouring point to (xPoint, yPoint)
          list.push(k);
        }
      }
      // clearing the temporary vector
      tempVec.clear();
    }
  }
}

std::queue<std::vector<int>> BFS::computeBFS(
    const std::queue<std::vector<int> >& frontierQueue,
    std::vector<int> start) {
  // Initializing the start x and y points
  int xPos = start[0];
  int yPos = start[1];
  std::queue<std::vector<int>> frontier, temp, list;
  std::vector<std::vector<int>> tempVector;
  // temporary vector variables
  std::vector<int> v, i, t, d;
  // Pushing origin to list
  d.push_back(0);
  d.push_back(0);
  list.push(d);
  temp = frontierQueue;
  // taking values from queue and passing it to tampVector
  while (!temp.empty()) {
    t = temp.front();
    tempVector.push_back(t);
    temp.pop();
  }
  // Pushing the start point to the queue
  pQueue.push(start);
  // loop runs until priority is queue is empty
  while (!pQueue.empty()) {
    v = pQueue.front();
    pQueue.pop();
    frontier.push(v);
    // checking the neighbour list
    neighbourList(tempVector, v, list);
    // loop runs for all neighbour
    while (!list.empty()) {
      i = list.front();
      list.pop();
      // checking if the point is already explored or not
      if (explored[i[0]][i[1]] == 0) {
        // setting the explored
        explored[i[0]][i[1]] = 1;
        distance[i[0]][i[1]] = distance[v[0]][v[1]] + 1;
        // pushing the value to the priority queue
        pQueue.push(i);
      }
    }
  }
  return frontier;
}
BFS::~BFS() {
}
