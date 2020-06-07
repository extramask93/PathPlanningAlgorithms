//
// Created by damian on 10.05.2020.
//

#include "IPlanner.h"
std::string IPlanner::getName() const {return name_;}
IPlanner::IPlanner(std::shared_ptr<util::GridMap<unsigned char>> map, const std::string &name) : map_(map),name_(name) {}
bool IPlanner::isStartAndGoalValid(const util::Point &start, const util::Point &goal)
{
    try {
        auto s = map_->worldToMap(start);
        auto g = map_->worldToMap(goal);
        if(!map_->isFree(s) || !map_->isFree(g)) {
            return false;
        }
        if(s == g) {
            return false;
        }
    } catch (util::OutOfBoundsException ex) {
        return false;
    }
    return true;

}
void IPlanner::initialize2(std::shared_ptr<util::GridMap<unsigned char>> map, const util::Options &options)
{
    map_ = map;
    options_ = options;
}
void IPlanner::setMap(std::shared_ptr<util::GridMap<unsigned char>> map)
{
    map_ = map;
}
util::GridMap<unsigned char> *IPlanner::getMap()
{
    return map_.get();
}
