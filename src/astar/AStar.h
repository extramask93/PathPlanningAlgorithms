//
// Created by damian on 01.05.2020.
//

#ifndef PLANNING_ASTAR_H
#define PLANNING_ASTAR_H
#include "Point.h"
#include "GridMap.h"
#include <vector>
namespace astar {
class AStar
{
  public:
    enum class HeuristicType { EUCLID,
        MANHATTAN,
        NO_HEURISTIC };
    explicit AStar(const util::GridMap<int> &map, HeuristicType heuristic = HeuristicType::EUCLID);
    std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal);
    void setHeuristic(AStar::HeuristicType heuristic);
    HeuristicType getHeuristic() const;

  protected:
    bool isObstacle(const util::Location &location) const;
    std::vector<int> getNeighborIndexes(int index) const;
    double getMoveCost(int currentIndex, int neighborIndex) const;
    double getHeuristicCost(int currentIndex, int goalCell) const;

  private:
    util::GridMap<int> obstacleMap_;
    HeuristicType currentHeuristic_;
    int startIndex_;
    int goalIndex_;
};
}// namespace astar


#endif//PLANNING_ASTAR_H
