#ifndef UTILS_H
#define UTILS_H
#include <vector>
#include <iostream>

class Node
{
    // Variables used here are constantly accessed and checked; leaving public for now.
  public:
    int x_;
    int y_;
    int id_;
    int pid_;
    double cost_;
    double h_cost_;

    Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0);

    Node operator+(Node p);

    Node operator-(Node p);

    void operator=(Node p);

    bool operator==(Node p);

    bool operator!=(Node p);
};


struct compare_cost
{

    bool operator()(Node &p1, Node &p2);
};

std::vector<Node> GetMotion();


#endif
