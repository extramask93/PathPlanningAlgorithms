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
    for (unsigned iteration = 0; iteration < maxNrOfIterations_; iteration++) {
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

util::Vertex rrt::RrtPlanner::getRandomVertex() const
{
    static std::mt19937 randomGenerator(std::random_device{}());
    static unsigned callCounter=0;
    using Distribution = std::uniform_real_distribution<double>;
    static Distribution xDistribution(obstacleMap_.getOriginX(),
        obstacleMap_.getOriginX() + obstacleMap_.getWorldWidth());
    static Distribution yDistribution(obstacleMap_.getOriginY(),
        obstacleMap_.getOriginY() + obstacleMap_.getWorldHeight());
    // roll goal with 5%-10% probability -> https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf
    if(callCounter % 10) {
        callCounter++;
        return util::Vertex(goalVertex_.getLocation(),util::Vertex::NO_ID, util::Vertex::NO_PARENT);
    }
    return util::Vertex{ util::Point(xDistribution(randomGenerator), yDistribution(randomGenerator)), util::Vertex::NO_ID, util::Vertex::NO_PARENT };
}

rrt::RrtPlanner::RrtPlanner(const util::GridMap<unsigned char> &obstacleMap) : goalVertex_(util::Point{ 0, 0 }),
                                                                     startVertex_(util::Point{ 0, 0 }),
                                                                     obstacleMap_(obstacleMap)
{
    maxNrOfIterations_ = 2000;
    maxExtendDistance_ = obstacleMap.getCellWidth() * obstacleMap.getResolution() * 0.1;
    goalRadius_ = 1.0;
}

util::Vertex rrt::RrtPlanner::steer(const util::Vertex &from, const util::Vertex &to) const
{
    double xFrom = from.getLocation().x;
    double yFrom = from.getLocation().y;
    double xTo = to.getLocation().x;
    double yTo = to.getLocation().y;
    // get the angle between the random point and our closest point (in rads)

    double distanceBetweenNodes = obstacleMap_.worldDistanceEuclidean(from.getLocation(), to.getLocation());
    double maxExtend = getRandomExtendDistance();
    if (distanceBetweenNodes > maxExtend/*maxExtendDistance_*/) {
        distanceBetweenNodes = maxExtend/*maxExtendDistance_*/;
    }

    util::Vertex newVertex(from.getLocation(), vertexList_.size(), from.getIndex());
    newVertex.addToPath(from.getLocation());
    double angleBetweenNodes = atan2(yTo - yFrom, xTo - xFrom);
    int maxNumberOfExpandSteps = floor(distanceBetweenNodes / obstacleMap_.getResolution());
    for (int i = 0; i < maxNumberOfExpandSteps; i++) {
        double xIncrement = newVertex.getX() + obstacleMap_.getResolution() * cos(angleBetweenNodes);
        double yIncrement = newVertex.getY() + obstacleMap_.getResolution() * sin(angleBetweenNodes);
        auto newPoint = util::Point{ xIncrement, yIncrement };
        if (isObstacle(newPoint)) {
            break;
        }
        newVertex.setLocation(newPoint);
        newVertex.addToPath(newPoint);
    }
    distanceBetweenNodes = obstacleMap_.worldDistanceEuclidean(newVertex.getLocation(), to.getLocation());
    if (distanceBetweenNodes <= obstacleMap_.getResolution()) {
        newVertex.addToPath(to.getLocation());
    }

    return newVertex;
}

bool rrt::RrtPlanner::isObstacle(const util::Point &point) const
{
    auto mapLocation = obstacleMap_.worldToMap(point);
    return obstacleMap_[mapLocation] == OBSTACLE;
}

util::Vertex rrt::RrtPlanner::findClosestVertex(const util::Vertex &from) const
{
    double closestDistance = std::numeric_limits<double>::infinity();
    util::Vertex closestVertex = vertexList_.at(0);
    for (const util::Vertex &vertex : vertexList_) {
        double distanceToNewVertex = obstacleMap_.worldDistanceEuclidean(from.getLocation(), vertex.getLocation());
        if (distanceToNewVertex < closestDistance) {
            closestDistance = distanceToNewVertex;
            closestVertex = vertex;
        }
    }

    return vertexList_.at(closestVertex.getIndex());
}

bool rrt::RrtPlanner::reachedGoal(const util::Vertex &vertex) const
{
    double distance = obstacleMap_.worldDistanceEuclidean(vertex.getLocation(), goalVertex_.getLocation());
    if (distance <= goalRadius_)
        return true;
    else
        return false;
}

bool rrt::RrtPlanner::isOnCollisionPath(const util::Vertex &vertex) const
{
    for (unsigned i = 0; i < vertex.getPath().size(); i++) {
        auto mapLocation = obstacleMap_.worldToMap(vertex.getPath().at(i));
        if (obstacleMap_[mapLocation] == OBSTACLE) {
            return true;
        }
    }
    return false;
}

void rrt::RrtPlanner::setMaxExtendDistance(double distance)
{
    maxExtendDistance_ = distance;
}

void rrt::RrtPlanner::setMaxNrOfIterations(unsigned int iterationNr)
{
    maxNrOfIterations_ = iterationNr;
}

std::vector<util::Point> rrt::RrtPlanner::buildPlan(int goalIndex) const
{
    // The plan we'll be adding to and returning
    std::vector<util::Point> plan;
    // no plan found
    if (goalIndex == -1)
        return plan;

    // The list of vertex indices we pass through to get to the goal
    std::deque<int> indexPath;
    int current_index = goalIndex;
    const int INDEX_OF_START_VERTEX = 0;

    while (current_index > INDEX_OF_START_VERTEX) {
        indexPath.push_front(current_index);
        current_index = vertexList_.at(current_index).getParentIndex();
    }
    indexPath.push_front(INDEX_OF_START_VERTEX);

    for (int i : indexPath) {
        util::Point pos = vertexList_[i].getLocation();
        plan.push_back(pos);
    }
    return plan;
}

std::vector<util::Point> rrt::RrtPlanner::makePlan(const util::Point &start, const util::Point &goal)
{
    auto goalIndex = makeGraph(start, goal);
    auto plan = buildPlan(goalIndex);
    return plan;
}
double rrt::RrtPlanner::getRandomExtendDistance() const
{
    static std::mt19937 randomGenerator(std::random_device{}());
    using Distribution = std::uniform_real_distribution<double>;
    static Distribution distribution(obstacleMap_.getResolution(),
                                      obstacleMap_.getCellWidth() * obstacleMap_.getResolution() * 0.1);
    return distribution(randomGenerator);
}
