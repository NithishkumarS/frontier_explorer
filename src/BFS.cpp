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

BFS::~BFS() {}
