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
#include <vector>
#include <queue>
#include "../include/BFS.hpp"

TEST(computeBFS, checkBFSAlogrithm) {
  // Creating a bfs object
  BFS bfsObj(3, 3);
  // Creating a queue for frontier points and output
  std::queue<std::vector<int>> frontierQueue, outputQueue;
  // Creating a vector for starting point
  std::vector<int> start, tempVec;

  // setting the starting position as (0,0)
  start.push_back(0);
  start.push_back(0);

  // setting the frontier points at (2,0) and (1,2)
  tempVec.push_back(0);
  tempVec.push_back(2);
  // Pushing the tempStart to the queue
  frontierQueue.push(tempVec);
  tempVec.clear();
  // setting the frontier points at (2,0) and (0,2)
  tempVec.push_back(1);
  tempVec.push_back(2);
  // Pushing the tempStart to the queue
  frontierQueue.push(tempVec);
  tempVec.clear();

  // Calling the computeBFS function
  outputQueue = bfsObj.computeBFS(frontierQueue, start);

  // Checking equality
  EXPECT_EQ(outputQueue.size(), frontierQueue.size());
}
TEST(NeighbourList, checkNeighbourListOutput) {
  // Creating a bfs object
  BFS bfsObj(3, 3);
  // Creating a queue for list of neighbour
  std::queue<std::vector<int>> list;
  // Creating a vector for starting point
  std::vector<int> start, tempVec;
  // Creating a vector for frontier List
  std::vector<std::vector<int>> frontierVector;

  // setting the starting position as (0,0)
  start.push_back(0);
  start.push_back(0);

  // setting the frontier points at (2,0) and (1,2)
  tempVec.push_back(0);
  tempVec.push_back(1);
  // Pushing the tempStart to the queue
  frontierVector.push_back(tempVec);
  tempVec.clear();
  // setting the frontier points at (2,0) and (0,2)
  tempVec.push_back(1);
  tempVec.push_back(0);
  // Pushing the tempStart to the queue
  frontierVector.push_back(tempVec);
  tempVec.clear();

  // Calling the neighbourList function
  bfsObj.neighbourList(frontierVector, start, list);

  // Checking equality
  EXPECT_EQ(2, list.size());
}

TEST(getExplored, checkReturnValue) {
  BFS bfsObj(3, 3);
  EXPECT_EQ(0, bfsObj.getExplored(0, 0));
}

TEST(getDistance, checkReturnValue) {
  BFS bfsObj(3, 3);
  EXPECT_EQ(0, bfsObj.getDistance(0, 0));
}
