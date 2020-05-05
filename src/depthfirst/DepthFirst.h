//
// Created by damian on 05.05.2020.
//

#ifndef PLANNING_DEPTHFIRST_H
#define PLANNING_DEPTHFIRST_H
#include "Point.h"
#include "GridMap.h"

namespace df {
class DepthFirst
{
  public:
    explicit DepthFirst(const util::GridMap<unsigned char> &map);
    std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal);

  protected:
    bool isObstacle(const util::Location &location) const;

  private:
    util::GridMap<unsigned char> obstacleMap_;
    int startIndex_;
    int goalIndex_;
};
}



#endif//PLANNING_DEPTHFIRST_H
