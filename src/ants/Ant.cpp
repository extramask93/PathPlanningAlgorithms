//
// Created by damian on 02.05.2020.
//

#include <Location.h>
#include "Ant.h"

namespace ants {


util::Node<int>& Ant::getCurrentNode() {
    return currentNode_;
}

void Ant::setCurrentNode(const util::Node<int> &currentNode) {
    currentNode_ = currentNode;
}

bool Ant::foundGoal() const {
    return foundGoal_;
}

void Ant::setFoundGoal(bool foundGoal) {
    foundGoal_ = foundGoal;
}

int Ant::getId() const {
    return id_;
}

void Ant::setId(int id) {
    id_ = id;
}

std::vector<util::Node<int>>& Ant::getPath() {
    return visitedNodes_;
}

void Ant::setPath(const std::vector<util::Node<int>> &path) {
    path_ = path;
}

util::Node<int> Ant::getPreviousNode() const {
    return previousNode_;
}

void Ant::setPreviousNode(const util::Node<int> &previousNode) {
    previousNode_ = previousNode;
}

int Ant::getSteps() const {
    return steps_;
}

void Ant::removeLoop() {
    for(auto nodeIterator = getPath().begin(); nodeIterator != getPath().end(); ++nodeIterator) {
        auto indices = getCoincidenceIndices(*nodeIterator);
        auto iter = indices.end();
        auto endIter = indices.begin();
        std::advance(endIter,1);
        while(iter > endIter) {
            iter--;
            auto visitedIterator = visitedNodes_.begin();
            std::advance(visitedIterator,*iter);
            visitedNodes_.erase(visitedIterator);
        }
    }
}
std::vector<int> Ant::getCoincidenceIndices(const util::Node<int>& node) {
    auto offset = visitedNodes_.begin();
    std::vector<int> indices;
    while (true) {
        offset = std::find_if(offset,visitedNodes_.end(),[&](const auto &a){ return (node.x == a.x) && (node.y == a.y);});
        if(offset == visitedNodes_.end()) {
            break;
        }
        auto index = std::distance(visitedNodes_.begin(),offset);
        indices.push_back(index);
        std::advance(offset,1);
    }
    return indices;
}
void Ant::setSteps(int steps) {
    steps_ = steps;
}

Ant::Ant(util::Node<int> start, util::Node<int> goal): currentNode_(start), finalNode_(goal) {
}

void Ant::addToPath(const util::Node<int> &node) {
    path_.push_back(node);
}

void Ant::moveTo(const util::Node<int> &nextNode) {
    currentNode_ = nextNode;
    rememberVisitedNode(nextNode);
}
util::Location Ant::getCurrentLocation() const
{
   return util::Location(currentNode_.x,currentNode_.y);
}
void Ant::rememberVisitedNode(const util::Node<int> &node)
{
    visitedNodes_.push_back(node);
}
void Ant::checkIfGoalReached()
{
    if(currentNode_ == finalNode_) {
        foundGoal_ = true;
    }
}

} /* namespace ants */
