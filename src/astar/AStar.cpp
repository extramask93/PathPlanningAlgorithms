//
// Created by damian on 01.05.2020.
//

#include <set>
#include "AStar.h"
namespace astar {
struct Node
{
    Node(int idx, double c) : index(idx), cost(c) {}
    explicit Node() : index(-1), cost(0.0) {}
    friend bool operator<(const Node &lhs, const Node &rhs)
    {
        return lhs.cost < rhs.cost;
    }
    double cost;
    int index;
};
AStar::AStar(const util::GridMap<int> &map, HeuristicType heuristic) : obstacleMap_(map), currentHeuristic_(heuristic),
                                                                       goalIndex_(-1),startIndex_(-1)
{
}
std::vector<util::Point> AStar::makePlan(const util::Point &start, const util::Point &goal)
{
    startIndex_ = obstacleMap_.mapToIndex(obstacleMap_.worldToMap(start));
    goalIndex_ = obstacleMap_.mapToIndex(obstacleMap_.worldToMap(goal));
    std::multiset<Node> priorityCosts;
    std::vector<double> gCosts(obstacleMap_.getCellWidth() * obstacleMap_.getCellHeight(), std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(gCosts.size(), -1);
    gCosts[startIndex_] = 0.0;
    Node currentNode(startIndex_, 0.0);
    priorityCosts.insert(currentNode);
    while (!priorityCosts.empty() && cameFrom[goalIndex_] == -1) {
        currentNode = *priorityCosts.begin();
        priorityCosts.erase(priorityCosts.begin());
        std::vector<int> neighborIndexes = getNeighborIndexes(currentNode.index);
        for (int i = 0; i < neighborIndexes.size(); i++) {
            if (cameFrom[neighborIndexes[i]] == -1) {
                gCosts[neighborIndexes[i]] = gCosts[currentNode.index]
                                             + getMoveCost(currentNode.index, neighborIndexes[i]);
                Node nextNode;
                nextNode.index = neighborIndexes[i];
                nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristicCost(neighborIndexes[i], goalIndex_);
                cameFrom[neighborIndexes[i]] = currentNode.index;
                priorityCosts.insert(nextNode);
            }
        }
    }
    if (cameFrom[goalIndex_] == -1 || startIndex_ == goalIndex_) {
        return std::vector<util::Point>{};
    }
    std::vector<util::Point> bestPath;
    currentNode.index = goalIndex_;
    while (currentNode.index != startIndex_) {
        auto loc = obstacleMap_.indexToMap(cameFrom[currentNode.index]);
        bestPath.push_back(obstacleMap_.mapToWorld(loc));
        currentNode.index = cameFrom[currentNode.index];
    }
    reverse(bestPath.begin(), bestPath.end());
    return bestPath;
}
std::vector<int> AStar::getNeighborIndexes(int index) const
{
    std::vector<int> neighborIndexes;
    auto motionModel = util::Robot::getMotionModel();
    for (const auto &motion : motionModel) {
        auto currentLocation = obstacleMap_.indexToMap(index);
        currentLocation = currentLocation + motion;
        if (obstacleMap_.isWithinMapBounds(currentLocation) && !isObstacle(currentLocation)) {
            neighborIndexes.push_back(obstacleMap_.mapToIndex(currentLocation));
        }
    }
    return neighborIndexes;
}
double AStar::getMoveCost(int currentIndex, int neighbourIndex) const
{
    auto currentLocation = obstacleMap_.indexToMap(currentIndex);
    auto neighborLocation = obstacleMap_.indexToMap(neighbourIndex);
    auto cost = obstacleMap_.distanceManhattan(currentLocation, neighborLocation);
    return cost;
}
double AStar::getHeuristicCost(int currentIndex, int goalIndex) const
{
    auto currentLocation = obstacleMap_.indexToMap(currentIndex);
    auto goalLocation = obstacleMap_.indexToMap(goalIndex);
    if (currentHeuristic_ == HeuristicType::EUCLID) {
        return obstacleMap_.distanceEuclidean(currentLocation, goalLocation);
    }
    if (currentHeuristic_ == HeuristicType::MANHATTAN) {
        return obstacleMap_.distanceManhattan(currentLocation, goalLocation);
    }
    return 0.0;
}
void AStar::setHeuristic(AStar::HeuristicType heuristic)
{
    currentHeuristic_ = heuristic;
}
AStar::HeuristicType AStar::getHeuristic() const
{
    return currentHeuristic_;
}
bool AStar::isObstacle(const util::Location &location) const
{
    return obstacleMap_[location] == 1;
}
}// namespace astar