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
 *
 * @brief To declare a class which implements BFS
 *
 * @driver Nantha Kumar Sunder
 * @navigator Nithish Sanjeev Kumar
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#pragma once
#include "ros/ros.h"
#include <vector>
#include <queue>

/** @brief Class to implement the control of turtlebot
 *  @param nh NodeHandle for the node
 *  @param depth Class object of depthData
 *  @param vel Publisher object
 *  @param laser Subsriber object
 *  @param info  variable to send velocities
 *  @return bool
 */
class BFS{
 private:
  std::queue< std::vector<int> > pQueue;
  std::vector < std::vector <int> > distance;
  std::vector < std::vector <int> > explored;
 public:
 /**@brief constructor
  * @param none
  * @return none
  */
BFS(int, int);
std::queue<std::vector<int>> computeBFS(const std::queue<std::vector<int>>&, std::vector<int>);
void neighbourList(const std::vector<std::vector<int> >& ,
                    std::vector<int> , std::queue<std::vector<int> >& );
int getExplored(int, int);
int getDistance(int, int);

/**@brief destructor
  * @param zStart
  * @return none
  */
~BFS();
};
