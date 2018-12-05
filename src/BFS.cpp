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


BFS::~BFS() {}
