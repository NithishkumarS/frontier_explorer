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

/**@file explore.hpp
 *
 * @brief a class to explores the map
 *
 * @driver Nantha Kumar Sunder
 * @navigator Nithish Sanjeev Kumar
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#pragma once
#include <queue>
#include <vector>
#include "map.hpp"

/**
 *  @brief Class to implement the BFS
 *
 *  @param path vector to store the path
 *
 *  @return None
 */
class explore {
 private:
  // vector to store the path
  std::vector<std::vector<int>> path;

 public:
  /**
   *  @brief class constructor
   *
   *  @return None
   */
  explore();
  /**
   *  @brief pathSearch to find the path to the goal
   *
   *  @param start contains the start point for the path
   *  @param end contains the end point for the path
   *  @param map object to get rows and columns
   *
   *  @return None
   */
  void pathSearch(std::vector<int>, std::vector<int>, map&);
  /**
   *  @brief function to compute the shortest path using BFS
   *
   *  @param start vector contains the start point for the path
   *  @param end vector contains the end point for the path
   *  @param path that stores the path to the goal
   *  @param map Object to get the rows, grid value and column
   *
   *  @return bool
   */
  bool computeShortestPath(std::vector<int>, std::vector<int>,
                           std::vector<std::vector<int> >&, map&);
  /**
   *  @brief function to find the neighbour of the given point which is without the obstacle
   *
   *  @param v vector contains the point to which neighbour has to be found
   *  @param list is a queue contains the list of neighbour
   *  @param map Object to get rows and columns
   *
   *  @return None
   */
  void findObstacleFreeNeighbours(std::vector<int>,
                                  std::queue<std::vector<int> >&, map&);
  /**
   *  @brief function to publish goal to the move base topic
   *
   *  @param resolution is a float that contains the resolution of the map
   *  @param map Object to get the rows, column
   *
   *  @return bool whether it has reached the target or not
   */
  bool navigate(float, map&, std::vector<int>);
  /**
   *  @brief class destructor
   *
   *  @return None
   */
  ~explore();
};
