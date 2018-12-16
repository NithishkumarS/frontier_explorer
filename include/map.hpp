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

/**@file map.hpp
 *
 * @brief To declare a class which implements the map and processes the frontier
 *
 * @driver Nithish Sanjeev Kumar
 * @navigator Nantha Kumar Sunder
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#pragma once

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <queue>
#include "../include/BFS.hpp"

/**
 * @brief Class to implement the MAP
 *  @param frontier queue to hold the frontier points
 *  @param Centroid queue to hold the centroid of
 *         individual frontiers
 *  @param grid the map as 2d vector
 *  @param row integer with row size of grid
 *  @param cols integer with coloumn size of grid
 *  @param mapResolution integer with resolution of grid
 *  @param OGM_Subscriber is the subscriber handle for /Map
 *  @param start vector containing the origin
 *  @return none
 */
class map {
 private:
  std::queue<std::vector<int> > frontierQueue;
  std::queue<std::vector<int> > centroidQueue;
  std::queue<std::vector<int> > explored;
  std::vector<std::vector<int> > grid;
  int rows, cols;
  float mapResolution;
  ros::Subscriber OGM_subscriber;

 public:
  /**
   *  @brief Constructor for the class map
   *  @param None
   *  @return None
   */
  map();

  std::vector<int> start;

  /**
   *  @brief Function to search frontiers in the map
   *  @param X coordinate of grid
   *  @param Y coordinate of grid
   *  @return none
   */
  void frontierSearch(int, int);

  /**
   *  @brief Function to check if the neighbour's grid value
   *  @param X coordinate of grid
   *  @param Y coordinate of grid
   *  @param Value to be checked
   *  @return Check
   */
  bool checkNeighbour(int, int, int);

  /**
   *  @brief Function to compute the centroid of the frontier
   *  @param Frontier Queue containing the centroid coordinates
   *  @return Centroid point
   */
  std::vector<int> computeFrontierCentroid(const std::queue<std::vector<int>>&);

  /**
   *  @brief Function to find the point closest to computed centroid
   *  @param Frontier Queue containing individual frontier points
   *  @return Point on frontier nearest to centroid
   */
  std::vector<int> nearest(const std::queue<std::vector<int>>&, int, int);

  /**
   *  @brief Function to request map from gmapping node
   *  @param Node handle
   *  @return Check if map is received
   */
  bool requestMap(ros::NodeHandle&);

  /**
   *  @brief CallBack function for the map subscriber
   *  @param Contains the map in as occupancy grid in row major form
   *  @return none
   */
  void mapcallback(const nav_msgs::OccupancyGrid& msg);

  /**
   *  @brief Function to find the point closest to computed centroid
   *  @param Frontier Queue containing individual frontier points
   *  @return Point on frontier nearest to centroid
   */
  std::vector<int> nearestCentroid(int, int);

  /**
   *  @brief Function to return the grid value
   *  @param X coordinate
   *  @param Y coordinate
   *  @return Grid value
   */
  int getGridValue(int, int);

  /**
   *  @brief Function to return the size of centroid queue
   *  @param None
   *  @return centroid size
   */
  int getCentroidSize();

  /**
   *  @brief Function to return row size
   *  @param None
   *  @return row size
   */
  int returnRows();

  /**
   *  @brief Function to return coloumn size
   *  @param None
   *  @return coloumn size
   */
  int returnCols();

  /**
   *  @brief Function to return resolution of the map
   *  @param None
   *  @return resolution of the map
   */
  float getResolution();

  /**
   *  @brief Function to return origin
   *  @param None
   *  @return origin vector
   */
  std::vector<int> getStart();

  /**
   *  @brief Destructor for the class map
   *  @param None
   *  @return None
   */
  ~map();
};
