//
// Created by damian on 26.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_NODE_H
#define POTENTIALFIELDSPROJECT_NODE_H
namespace util {
struct Node
{
    explicit Node(int idd, double xx, double yy, double c = 0.0, int pid = -1) : id(idd), x(xx), y(yy), cost(c), parent(pid), hCost(0) {}
    bool operator==(const util::Node &node)
    {
        return this->id == node.id;
    }
    double x;
    double y;
    double cost;
    double hCost;
    int parent;
    int id;
};
struct compare_cost
{
    bool operator()(const util::Node &p1, const util::Node &p2)
    {
        if (p1.cost + p1.hCost > p2.cost + p2.hCost) {
            return true;
        } else if (p1.cost + p1.hCost == p2.cost + p2.hCost && p1.hCost >= p2.hCost) {
            return true;
        }
        return false;
    }
};
}// namespace util
#endif//POTENTIALFIELDSPROJECT_NODE_H
