//
// Created by damian on 10.05.2020.
//

#ifndef PLANNING_IPLANNER_H
#define PLANNING_IPLANNER_H
#include "GridMap.h"
#include "Point.h"
#include "Options.h"
#include <vector>

class IPlanner
{
  public:
    virtual void initialize(const util::GridMap<unsigned char> &map,const util::Options &options = util::Options{}) = 0;
    virtual std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal) = 0;
    virtual ~IPlanner() {}
};


#endif//PLANNING_IPLANNER_H
