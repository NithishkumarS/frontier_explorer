#include <ros/ros.h>
#include <vector>
#include "BFS.hpp"
#include <queue>

BFS::BFS(int rows, int cols) {
  explored.resize(rows);
  for (auto& i : explored){
        i.resize(cols);
        for(auto& j :i)
          j =0;
       }
  distance.resize(rows);
    for (auto& i : distance){
          i.resize(cols);
          for(auto& j :i)
            j =0;
         }
}

int BFS::getExplored(int x, int y) {
 return explored[x][y];
}

int BFS::getDistance(int x, int y) {
  return distance[x][y];
}

void BFS::neighbourList(const std::vector<std::vector<int>>& frontierVector ,
                    std::vector<int> v , std::queue<std::vector<int> >& list ){
  int xPoint = v[0];
  int yPoint = v[1];
  std::vector<int> tempVec;
  int move[3] = {1, -1 ,0};


  for (auto & x : move) {
      for (auto & y : move) {
            tempVec.push_back(yPoint+y);
            tempVec.push_back(xPoint+x);
            for (auto & k : frontierVector) {
                   if (k == tempVec) {
                     list.push(k);
                   }
             }
        }
  }
}

std::queue<std::vector<int>> BFS::computeBFS(const std::queue<std::vector<int> >& frontierQueue, std::vector<int> start) {
  int xPos = start[0];
  int yPos = start[1];
  std::queue<std::vector<int>> frontier,temp,list;
  std::vector<std::vector<int>> tempVector;
  std::vector<int> v, i, t;
  temp = frontierQueue;

  while(!temp.empty()) {
      t = temp.front();
      tempVector.push_back(t);
      temp.pop();
  }
  pQueue.push(start);

  while (!pQueue.empty()){
               v = pQueue.front();
               pQueue.pop();
               frontier.push(v);
               neighbourList(tempVector,v,list);
               while(!list.empty()){
                   i = list.front();
                   list.pop();
                   if (explored[i[0]][i[1]] == 0) {
                     explored[i[0]][i[1]] = 1;
                     distance[i[0]][i[1]] = distance[v[0]][v[1]] + 1;
                     pQueue.push(i);
                   }
               }
  }
  return frontier;
}

BFS::~BFS() {}
