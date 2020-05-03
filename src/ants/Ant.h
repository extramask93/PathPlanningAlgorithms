//
// Created by damian on 02.05.2020.
//

#ifndef PLANNING_ANT_H
#define PLANNING_ANT_H
#include <vector>
#include "Node.h"
namespace ants {
class Ant
{
  public:
    Ant(util::Node<int> start, util::Node<int> goal);
    util::Node<int>& getCurrentNode();
    util::Location getCurrentLocation() const;
    void setCurrentNode(const util::Node<int> &currentNode);
    void moveTo(const util::Node<int> &nextNode);
    void removeLoop();
    bool foundGoal() const;
    void setFoundGoal(bool foundGoal);
    int getId() const;
    void setId(int id);
    std::vector<util::Node<int>> &getPath();
    void addToPath(const util::Node<int> &node);
    void setPath(const std::vector<util::Node<int>> &path);
    util::Node<int> getPreviousNode() const;
    void setPreviousNode(const util::Node<int> &previousNode);
    int getSteps() const;
    void setSteps(int steps);
    void rememberVisitedNode(const util::Node<int> &node);
    void checkIfGoalReached();
    std::vector<util::Node<int>> visitedNodes_;
    std::vector<int> getCoincidenceIndices(const util::Node<int>& node) ;
  private:
    std::vector<util::Node<int>> path_;
    util::Node<int> finalNode_;
    bool foundGoal_ = false;
    util::Node<int> currentNode_;
    util::Node<int> previousNode_;
    int steps_ = 0;
    int id_ = 0;
};
}

#endif//PLANNING_ANT_H
