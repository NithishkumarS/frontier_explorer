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

/**@file BFS.hpp
 * @brief To declare a class which implements BFS
 * @driver Nantha Kumar Sunder
 * @navigator Nithish Sanjeev Kumar
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved
 */
#pragma once
#include <vector>
#include <queue>
#include "ros/ros.h"
/**
 *  @brief Class to implement the BFS
 *
 *  @param pQueue queue to store the coordinates
 *  @param distance euclidean distance from the start node
 *  @param explored visited node in the occupancy grid
 *
 *  @return None
 */
class BFS {
 private:
  // Queue to store the priority queue for BFS
  std::queue<std::vector<int> > pQueue;
  // euclidean distance from the start node
  std::vector<std::vector<int> > distance;
  // variable to set the visited grid from BFS
  std::vector<std::vector<int> > explored;

 public:
  /**
   *  @brief Constructor for the class BFS
   *
   *  @param cols column to resize the explored and distance
   *  @param rows row to resize the explored and distance
   *
   *  @return None
   */
  BFS(int, int);
  /**
   *  @brief function to compute BFS
   *
   *  @param frontierQueue contains list of all frontiers
   *  @param start point of the turtlebot
   *
   *  @return frontiers Queue to store all the frontiers
   */
  std::queue<std::vector<int>> computeBFS(const std::queue<std::vector<int>>&,
                                          std::vector<int>);
  /**
   *  @brief function to estimate the neighbouring node
   *
   *  @param frontier vector
   *  @param vector coordinate to which neighbouring node has to found out
   *  @param list of all neighbouring node of the given coordinate
   *
   *  @return frontiers Queue to store all the frontiers
   */
  void neighbourList(const std::vector<std::vector<int> >&, std::vector<int>,
                     std::queue<std::vector<int> >&);
  /**
   *  @brief function to get the explored value of a specific grid
   *
   *  @param x x-position of the grid
   *  @param y y-position of the grid
   *
   *  @return integer value of explored grid for the x and y coordinates
   */
  int getExplored(int, int);
  /**
   *  @brief function to get the distance value of a specific grid
   *
   *  @param x x-position of the grid
   *  @param y y-position of the grid
   *
   *  @return integer value of distance grid for the x and y coordinates
   */
  int getDistance(int, int);
  /**
   *  @brief class destructor
   *
   *  @return None
   */
  ~BFS();
};
