
#include "Utils.h"
#include <random>
#include <iomanip>
Node::Node(int x, int y, double cost, double h_cost, int id, int pid)
{
    this->x_ = x;
    this->y_ = y;
    this->cost_ = cost;
    this->h_cost_ = h_cost;
    this->id_ = id;
    this->pid_ = pid;
}

Node Node::operator+(Node p)
{
    Node tmp;
    tmp.x_ = this->x_ + p.x_;
    tmp.y_ = this->y_ + p.y_;
    tmp.cost_ = this->cost_ + p.cost_;
    return tmp;
}

Node Node::operator-(Node p)
{
    Node tmp;
    tmp.x_ = this->x_ - p.x_;
    tmp.y_ = this->y_ - p.y_;
    return tmp;
}

void Node::operator=(Node p)
{
    this->x_ = p.x_;
    this->y_ = p.y_;
    this->cost_ = p.cost_;
    this->h_cost_ = p.h_cost_;
    this->id_ = p.id_;
    this->pid_ = p.pid_;
}

bool Node::operator==(Node p)
{
    if (this->x_ == p.x_ && this->y_ == p.y_) return true;
    return false;
}

bool Node::operator!=(Node p)
{
    if (this->x_ != p.x_ || this->y_ != p.y_) return true;
    return false;
}

bool compare_cost::operator()(Node &p1, Node &p2)
{
    // Can modify this to allow tie breaks based on heuristic cost if required
    if (p1.cost_ + p1.h_cost_ > p2.cost_ + p2.h_cost_)
        return true;
    else if (p1.cost_ + p1.h_cost_ == p2.cost_ + p2.h_cost_ && p1.h_cost_ >= p2.h_cost_)
        return true;
    return false;
}

std::vector<Node> GetMotion()
{
    Node down(0, 1, 1, 0, 0, 0);
    Node up(0, -1, 1, 0, 0, 0);
    Node left(-1, 0, 1, 0, 0, 0);
    Node right(1, 0, 1, 0, 0, 0);
    std::vector<Node> v;
    v.push_back(down);
    v.push_back(up);
    v.push_back(left);
    v.push_back(right);
    return v;
}
