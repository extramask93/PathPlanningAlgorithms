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
    int index;
    double cost;
};
AStar::AStar(std::shared_ptr<util::GridMap<unsigned char>> map, HeuristicType heuristic) : IPlanner(map), currentHeuristic_(heuristic),
                                                                       goalIndex_(-1),startIndex_(-1)
{
}
std::vector<util::Point> AStar::makePlan(const util::Point &start, const util::Point &goal)
{
    startIndex_ = static_cast<int>(map_->mapToIndex(map_->worldToMap(start)));
    goalIndex_ = static_cast<int>(map_->mapToIndex(map_->worldToMap(goal)));
    std::multiset<Node> priorityCosts;
    std::vector<double> gCosts(map_->getCellWidth() * map_->getCellHeight(), std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(gCosts.size(), -1);
    gCosts[static_cast<unsigned>(startIndex_)] = 0.0;
    Node currentNode(startIndex_, 0.0);
    priorityCosts.insert(currentNode);
    while (!priorityCosts.empty() && cameFrom[static_cast<unsigned long>(goalIndex_)] == -1) {
        /*if(currentHeuristic_ != HeuristicType::NO_HEURISTIC) {
            if( cameFrom[goalIndex_] != -1) {
                break;
            }
        }*/
        currentNode = *priorityCosts.begin();
        priorityCosts.erase(priorityCosts.begin());
        std::vector<int> neighborIndexes = getNeighborIndexes(currentNode.index);
        for (unsigned i = 0; i < neighborIndexes.size(); i++) {
            if (cameFrom[static_cast<unsigned long>(neighborIndexes[i])] == -1) {
                gCosts[static_cast<unsigned  long>(neighborIndexes[i])] = gCosts[static_cast<unsigned long>(currentNode.index)]
                                             + getMoveCost(currentNode.index, neighborIndexes[i]);
                Node nextNode;
                nextNode.index = neighborIndexes[i];
                nextNode.cost = gCosts[static_cast<unsigned  long>(neighborIndexes[i])] + getHeuristicCost(neighborIndexes[i], goalIndex_);
                cameFrom[static_cast<unsigned  long>(neighborIndexes[i])] = currentNode.index;
                priorityCosts.insert(nextNode);
            }
        }
    }
    if (cameFrom[static_cast<unsigned long>(goalIndex_)] == -1 || startIndex_ == goalIndex_) {
        return std::vector<util::Point>{};
    }
    std::vector<util::Point> bestPath;
    currentNode.index = goalIndex_;
    bestPath.push_back(goal);
    while (currentNode.index != startIndex_) {
        auto loc = map_->indexToMap(static_cast<unsigned int>(cameFrom[static_cast<unsigned  long>(currentNode.index)]));
        bestPath.push_back(map_->mapToWorld(loc));
        currentNode.index = cameFrom[static_cast<unsigned long>(currentNode.index)];
    }
    bestPath.push_back(start);
    reverse(bestPath.begin(), bestPath.end());
    return bestPath;
}
std::vector<int> AStar::getNeighborIndexes(int index) const
{
    std::vector<int> neighborIndexes;
    auto motionModel = util::Robot::getMotionModel();
    for (const auto &motion : motionModel) {
        auto currentLocation = map_->indexToMap(static_cast<unsigned int>(index));
        currentLocation = currentLocation + motion;
        if (map_->isWithinMapBounds(currentLocation) && !isObstacle(currentLocation)) {
            neighborIndexes.push_back(static_cast<int>(map_->mapToIndex(currentLocation)));
        }
    }
    return neighborIndexes;
}
double AStar::getMoveCost(int currentIndex, int neighbourIndex) const
{
    auto currentLocation = map_->indexToMap(static_cast<unsigned int>(currentIndex));
    auto neighborLocation = map_->indexToMap(static_cast<unsigned int>(neighbourIndex));
    auto cost = map_->distanceEuclidean(currentLocation, neighborLocation);
    return cost;
}
double AStar::getHeuristicCost(int currentIndex, int goalIndex) const
{
    auto currentLocation = map_->indexToMap(static_cast<unsigned int>(currentIndex));
    auto goalLocation = map_->indexToMap(static_cast<unsigned int>(goalIndex));
    if (currentHeuristic_ == HeuristicType::EUCLID) {
        return map_->distanceEuclidean(currentLocation, goalLocation);
    }
    if (currentHeuristic_ == HeuristicType::MANHATTAN) {
        return map_->distanceManhattan(currentLocation, goalLocation);
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
    return (*map_)[location] == 0;
}
void AStar::initialize(const util::GridMap<unsigned char> &map, const util::Options &options)
{
    (void)map;
    setHeuristic(static_cast<HeuristicType>(options.heuristic));
}
}// namespace astar