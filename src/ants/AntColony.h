//
// Created by damian on 02.05.2020.
//

#ifndef PLANNING_ANTCOLONY_H
#define PLANNING_ANTCOLONY_H
#include "Point.h"
#include "Location.h"
#include "GridMap.h"
#include "Ant.h"
#include "Node.h"
#include "Edge.h"
#include <boost/functional/hash.hpp>
#include <unordered_map>
namespace ants {
class AntColony
{
    using IDPair = std::pair<unsigned ,unsigned >;
  public:
    AntColony(const util::GridMap<int> &obstacleMap);
    std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal);
  private:
    void initializePheromonesAtEdges();
    util::Node<int>& selectNewNode(util::Node<int> &node);
    void deterioratePheromones();
    bool isObstacle(const util::Location &location) const;
  private:
    util::GridMap<int> obstacleMap_;
    std::vector<util::Node<int>> nodes_;
    util::Node<int> startNode_;
    util::Node<int> goalNode_;
    std::size_t numOfAnts_ = 100;
    std::vector<std::vector<util::Node<int>>> paths_;
    double alpha_ =1.0;
    double beta_ =1.0;
    double rewardScalingFactor_ =1.0;
    double evaporationRate_= 0.01;
    double initial_pheromone_level = 1.0;
    std::size_t maxIterations_ = 10;
    std::vector<Ant> ants_;
    int maxSteps_ =100;
};
}


#endif//PLANNING_ANTCOLONY_H
