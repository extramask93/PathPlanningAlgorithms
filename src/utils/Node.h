//
// Created by damian on 26.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_NODE_H
#define POTENTIALFIELDSPROJECT_NODE_H
#include "Edge.h"
namespace util {
template <typename CTYPE>
struct Node
{
    Node(){};
    explicit Node(int idd, CTYPE xx, CTYPE yy, double c = 0.0, int pid = -1) : id(idd), x(xx), y(yy), cost(c), parent(pid), hCost(0) {}
    void shiftByXY(CTYPE xx, CTYPE yy) {
        x += xx;
        y += yy;
    }
    bool isLocationEqual(const util::Node<CTYPE> &node) {
        return ((this->x == node.x) && (this->y == node.y));
    }
    bool operator==(const util::Node<CTYPE> &node)
    {
        return this->id == node.id;
    }
    CTYPE x;
    CTYPE y;
    std::vector<util::Edge> edges;
    double cost;
    double hCost;
    int parent;
    int id;
};
struct compare_cost
{
    bool operator()(const util::Node<double> &p1, const util::Node<double> &p2)
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
