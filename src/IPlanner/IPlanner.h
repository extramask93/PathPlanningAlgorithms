//
// Created by damian on 10.05.2020.
//

#ifndef PLANNING_IPLANNER_H
#define PLANNING_IPLANNER_H
#include "GridMap.h"
#include "Point.h"
#include "Options.h"
#include <string>
#include <vector>

class IPlanner
{
  public:
    IPlanner() = default;
    IPlanner(std::shared_ptr<util::GridMap<unsigned char>> map, const std::string &name = "");
    virtual void initialize(const util::GridMap<unsigned char> &map,const util::Options &options = util::Options{}) = 0;
    virtual void initialize2(std::shared_ptr<util::GridMap<unsigned char>> map,const util::Options &options = util::Options{});
    virtual std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal) = 0;
    virtual bool isStartAndGoalValid(const util::Point &start, const util::Point &goal);
    virtual std::string getName() const final;
    virtual void setMap(std::shared_ptr<util::GridMap<unsigned char>> map);
    virtual util::GridMap<unsigned char>* getMap();
    /*rule of five*/
    IPlanner(const IPlanner&) = default;
    IPlanner(IPlanner&&) = default;
    IPlanner& operator=(const IPlanner&) = default;
    IPlanner& operator=(IPlanner&&) = default;
    virtual ~IPlanner() = default;
    /*rule of five*/
  protected:
    std::shared_ptr<util::GridMap<unsigned char>> map_{nullptr};
    util::Options options_;
    std::string name_;
};


#endif//PLANNING_IPLANNER_H
