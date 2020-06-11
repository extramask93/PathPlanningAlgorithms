//
// Created by damian on 05.05.2020.
//

#include "DepthFirst.h"
#include <stack>
#include <unordered_map>
namespace df {
struct Node
{
    Node(int idx, int pidx) : id(idx), pid(pidx) {}
    explicit Node() : id(-1), pid(-1) {}
    int id;
    int pid;
};
df::DepthFirst::DepthFirst(std::shared_ptr<util::GridMap<unsigned char>> map) : IPlanner(map)
{
}
std::vector<util::Point> df::DepthFirst::makePlan(const util::Point &start, const util::Point &goal)
{
    startIndex_ = static_cast<int>(map_->mapToIndex(map_->worldToMap(start)));
    goalIndex_ = static_cast<int>(map_->mapToIndex(map_->worldToMap(goal)));
    std::stack<Node> openSet;
    std::unordered_map<int,Node> closedSet;
    Node currentNode(startIndex_, -1);
    Node goalNode(goalIndex_, -1);
    openSet.push(currentNode);
    bool goalFound = false;
    while (!openSet.empty()) {
        currentNode = openSet.top();
        openSet.pop();
        if(currentNode.id == goalIndex_) {
            goalNode.pid = currentNode.id;
            closedSet.insert(std::make_pair(goalNode.id, goalNode));
            goalFound = true;
            break;
        }
        for(const auto &motion : util::Robot::getMotionModel()) {
            util::Location currentNodeLocation = map_->indexToMap(static_cast<unsigned int>(currentNode.id));
            util::Location newLocation = currentNodeLocation + motion;
            if(map_->isWithinMapBounds(newLocation) && !isObstacle(newLocation)) {
                Node newNode(static_cast<int>(map_->mapToIndex(newLocation)),currentNode.id);
                if(closedSet.find(newNode.id) == closedSet.end()) {
                    openSet.push(newNode);
                    closedSet.insert(std::make_pair(newNode.id, newNode));
                }
            }
        }

    }
    std::vector<util::Point> bestPath;
    if(!goalFound) {
        return bestPath;
    }
    while (currentNode.id != startIndex_) {
        auto loc = map_->indexToMap(static_cast<unsigned int>(closedSet[currentNode.id].id));
        bestPath.push_back(map_->mapToWorld(loc));
        currentNode.id = closedSet[currentNode.id].pid;
    }
    bestPath.push_back(start);
    reverse(bestPath.begin(), bestPath.end());
    return bestPath;
}
bool DepthFirst::isObstacle(const util::Location &location) const
{
    return !map_->isFree(location);
}

}
