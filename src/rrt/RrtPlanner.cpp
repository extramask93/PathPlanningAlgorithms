//
// Created by damian on 01.05.2020.
//

#include <random>
#include <deque>
#include "RrtPlanner.h"

int rrt::RrtPlanner::makeGraph(const util::Point &start, const util::Point &goal)
{
    goalVertex_ = util::Vertex(goal, util::Vertex::NO_ID, util::Vertex::NO_PARENT);
    startVertex_ = util::Vertex(start, 0, util::Vertex::NO_PARENT);
    int goalIndex = -1;
    vertexList_.push_back(startVertex_);
    unsigned iteration = 0;
    for (; iteration < maxNrOfIterations_; iteration++) {
        auto randomVertex = getRandomVertex();
        auto closestVertex = findClosestVertex(randomVertex);
        auto newVertex = steer(closestVertex, randomVertex);
        if (isOnCollisionPath(newVertex)) {
            continue;
        }
        newVertex.setIndex(vertexList_.size());
        newVertex.setParentIndex(closestVertex.getIndex());
        vertexList_.push_back(newVertex);
        if (reachedGoal(newVertex)) {
            goalVertex_.setIndex(vertexList_.size());
            goalVertex_.setParentIndex(newVertex.getIndex());
            vertexList_.push_back(goalVertex_);
            goalIndex = goalVertex_.getIndex();
            break;
        }
    }
    return goalIndex;
}



rrt::RrtPlanner::RrtPlanner(std::shared_ptr<util::GridMap<unsigned char>> &map) : IPlanner(map), goalVertex_(util::Point{ 0, 0 }),
                                                                     startVertex_(util::Point{ 0, 0 })
{
    maxNrOfIterations_ = 10000;
    maxExtendDistance_ = map_->getCellWidth() * map_->getResolution() * 0.5;
    goalSamplingRatio_ = 0.1;
    goalRadius_ = 1.0;//std::ceil(obstacleMap.getCellWidth()*obstacleMap.getResolution()*0.01);
}



void rrt::RrtPlanner::setMaxExtendDistance(double distance)
{
    maxExtendDistance_ = distance;
}

void rrt::RrtPlanner::setMaxNrOfIterations(unsigned int iterationNr)
{
    maxNrOfIterations_ = iterationNr;
}



std::vector<util::Point> rrt::RrtPlanner::makePlan(const util::Point &start, const util::Point &goal)
{
    auto goalIndex = makeGraph(start, goal);
    auto plan = buildPlan(goalIndex);
    return plan;
}

void rrt::RrtPlanner::initialize(const util::GridMap<unsigned char> &map, const util::Options &options)
{
    (void)map;
    (void)options;
}
