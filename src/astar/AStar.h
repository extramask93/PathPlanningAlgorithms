//
// Created by damian on 01.05.2020.
//

#ifndef PLANNING_ASTAR_H
#define PLANNING_ASTAR_H
#include "Point.h"
#include "GridMap.h"
#include "IPlanner.h"
#include <vector>
namespace astar {
class AStar : public IPlanner
{
  public:
    enum class HeuristicType { EUCLID,
        MANHATTAN,
        NO_HEURISTIC };
    explicit AStar(std::shared_ptr<util::GridMap<unsigned char>> map, HeuristicType heuristic = HeuristicType::EUCLID);
    virtual std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal) override;
    virtual void initialize(const util::GridMap<unsigned char> &map, const util::Options &options = util::Options{}) override;
    void setHeuristic(AStar::HeuristicType heuristic);
    HeuristicType getHeuristic() const;

  protected:
    bool isObstacle(const util::Location &location) const;
    std::vector<int> getNeighborIndexes(int index) const;
    double getMoveCost(int currentIndex, int neighborIndex) const;
    double getHeuristicCost(int currentIndex, int goalCell) const;

  private:
    HeuristicType currentHeuristic_;
    int goalIndex_;
    int startIndex_;
};
}// namespace astar


#endif//PLANNING_ASTAR_H
