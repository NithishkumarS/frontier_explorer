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
 * @brief To implement the class function which explores the map
 *
 * @driver Nantha Kumar Sunder
 * @navigator Nithish Sanjeev Kumar
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */

#include "explore.hpp"

explore::explore() {}


void explore::findObstacleFreeNeighbours(std::vector<int> v , std::queue<std::vector<int> >& list, map& obj) {

  int xPoint = v[0];
  int yPoint = v[1];
  std::vector<int> tempVec;
  int move[3] = {1, -1 ,0};


    for (auto & x : move) {
        for (auto & y : move) {
            if( obj.getGridValue(xPoint+x,yPoint+y) != 1) {
              tempVec.push_back(yPoint+y);
              tempVec.push_back(xPoint+x);
              list.push(tempVec);
             }
        }
   }
}

bool explore::computeShortestPath( std::vector<int> start, std::vector<int> End, std::vector< std::vector<int> >& path, map& obj) {

 int goalFound = 0;

 std::vector < std::vector <std::vector<int> > > parent;
 std::queue< std::vector<int> > pQueue, list;
 std::vector < std::vector <int> > explored;
 std::vector<int> v, i, temp;

 int rows = obj.returnRows();
 int cols = obj.returnCols();

 parent.resize(rows);
 for (auto& i : parent){
     i.resize(cols);
     for (auto& j:i){
         j.resize(2);
         for(auto& k:j )
              k =0;
    }
 }

 explored.resize(rows);
 for (auto& i : explored){
     i.resize(cols);
     for(auto& j :i)
        j =0;
 }
 pQueue.push(start);

 while (!pQueue.empty()){
        v = pQueue.front();
        pQueue.pop();
        findObstacleFreeNeighbours(v, list, obj);

        while(!list.empty()){
            i = list.front();
            list.pop();
            if (explored[i[0]][i[1]] == 0) {
                explored[i[0]][i[1]] = 1;
                parent[i[0]][i[1]] = v;
                pQueue.push(i);
                if( End[0] == i[0] && End[1] == i[1]) {  //if its the goal point
                 goalFound =1;
                }
            }
        }
  }

  if(goalFound == 1) {

    path.push_back(End);
    temp = End;
    while(temp!=start) {
    temp = parent[temp[0]][temp[1]];
    path.push_back(temp);
    }
    goalFound = 0;
    return true;
  }
  return false;
}


void explore::pathSearch(std::vector<int> start, std::vector<int> end, map& obj) {
  bool check;
  std::vector <std::vector<int>> path;
  check = computeShortestPath(start, end, path, obj);
}

void explore::navigate(){
void pathToPose();
void generateVelocityCommands();
}

explore::~explore() {}
