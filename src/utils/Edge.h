//
// Created by damian on 02.05.2020.
//

#ifndef PLANNING_EDGE_H
#define PLANNING_EDGE_H
#include "Location.h"
namespace util {
struct Edge
{
    Edge(util::Location finalNode_, double probability_, double pheromone_) : finalNode(finalNode_), probability(probability_),
                                                                              pheromone(pheromone_){}
    Edge(){}
    util::Location finalNode;
    double probability = 0.0;
    double pheromone = 0.0;
};
}// namespace ants


#endif//PLANNING_EDGE_H
