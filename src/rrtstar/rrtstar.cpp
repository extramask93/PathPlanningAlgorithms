#include <chrono>
#include <algorithm>
#include <cstddef>
#include <random>
#include <deque>
#include "rrtstar.h"
#define OBSTACLE false
#define FREE false
#define MAX_DISTANCE_TRESHOLD 2.0
#define MAX_EXTEND_DISTANCE 1.0

namespace rrt {
using namespace std;
using namespace std::chrono;
RrtStar::RrtStar(std::shared_ptr<util::GridMap<unsigned char>> map)
    : IPlanner(map), goalVertex_(util::Point{ 0, 0 }), startVertex_(util::Point{ 0, 0 })
{
    gamma_ = 5;
    max_extend_distance_ = map_->getResolution() * 3;
}


std::vector<util::Point> RrtStar::makePlan(const util::Point &start,
    const util::Point &goal)
{
    current_iterations_ = 0;
    vertexList_.clear();
    util::Vertex startVertex(start);
    util::Vertex goalVertex(goal);
    goalVertex_ = util::Vertex(goal, util::Vertex::NO_ID, util::Vertex::NO_PARENT);
    startVertex_ = util::Vertex(start, 0, util::Vertex::NO_PARENT);
    start_ = start;
    goal_ = goal;
    int goal_index = RrtStar::findPath(startVertex, goalVertex);
    auto plan = buildPlan(goal_index);
    return plan;
}

int RrtStar::findPath(const util::Vertex &start, const util::Vertex &goal)
{
    int goal_index = -1;
    current_iterations_ = 0;
    vertexList_.clear();

    if (isObstacle(start) || isObstacle(goal)) {
        return goal_index;
    }

    util::Vertex root(start.getLocation(), 0, util::Vertex::NO_PARENT);
    vertexList_.push_back(root);
    while (current_iterations_ < max_iterations_) {
        util::Vertex randomVertex = getRandomVertex();
        if (isObstacle(randomVertex)) {
            continue;
        }
        const util::Vertex &closest_vertex = findClosestVertex(randomVertex);
        util::Vertex newVertex = steer(closest_vertex, randomVertex);
        newVertex.setParentIndex(closest_vertex.getIndex());
        if (isOnCollisionPath(newVertex)) {
            continue;
        }
        auto nearVerticesAndDistances = findNearVerticesIndexes(newVertex, max_distance_threshold_);
        std::vector<std::size_t> &nearVerticesIndices = nearVerticesAndDistances.first;
        std::vector<double> &nearDistances = nearVerticesAndDistances.second;
        chooseNewParent(newVertex, nearVerticesIndices, nearDistances);
        vertexList_.push_back(newVertex);
        rewire(newVertex, nearVerticesIndices, nearDistances);
        if (!runTillMaxIterations_ && reachedGoal(newVertex)) {

            util::Vertex goalVertex{ goal.getLocation(), static_cast<int>(vertexList_.size()), newVertex.getIndex() };
            vertexList_.push_back(goalVertex);
            goal_index = goalVertex.getIndex();
            break;
        }
        current_iterations_++;
    }
    if (runTillMaxIterations_) {
        auto closestVertex = findClosestVertex(goal);
        auto goalVertex = goal;
        goalVertex.setParentIndex(closestVertex.getIndex());
        goalVertex.setIndex(vertexList_.size());
        vertexList_.push_back(goalVertex);
        goal_index = goalVertex.getIndex();
        //find best node to connect to the goal
    }
    return goal_index;
}

void RrtStar::chooseNewParent(util::Vertex &newVertex, const std::vector<std::size_t> &nearVertcesIndxes, const std::vector<double> &nearVerticesDistances)
{
    if (nearVertcesIndxes.empty()) {
        auto parent = vertexList_[newVertex.getParentIndex()];
        newVertex.setCost(parent.getCost() + getDistance(newVertex, parent));
        return;
    }
    auto iteratorToMinDistance = std::min_element(nearVerticesDistances.begin(), nearVerticesDistances.end());
    std::size_t indexOfMinDistance = std::distance(nearVerticesDistances.begin(), iteratorToMinDistance);
    assert(("Near vertex was not found, probably extending farther than parent search distance", nearVertcesIndxes.size() > indexOfMinDistance));
    std::size_t indexOfClosestvertex = nearVertcesIndxes.at(indexOfMinDistance);
    util::Vertex &closestVertex = vertexList_.at(indexOfClosestvertex);
    newVertex.setCost(closestVertex.getCost() + nearVerticesDistances[indexOfMinDistance]);
    newVertex.setParentIndex(closestVertex.getIndex());
}
std::pair<std::vector<std::size_t>, std::vector<double>>
    RrtStar::findNearVerticesIndexes(const util::Vertex &newVertex, double maxDistanceTreshold)
{
    //TODO tune gamma for decaying neighbour selection
    //from
    //double gamma = 2 * (1 - 1 / 2); //or bigger
    //n is nr of nodes Algorithmic Foundations of Robotics XI
    double n = vertexList_.size();
    double intermediate = std::pow(std::log(n) / n, 0.5);
    double radius = gamma_ * intermediate;
    std::vector<std::size_t> nearVerticesIndexes{};
    std::vector<double> nearVerticesDistances{};
    for (const util::Vertex &vertex : vertexList_) {
        double distanceToNewVertex = getDistance(newVertex.getLocation(), vertex.getLocation());
        if (distanceToNewVertex <= radius /*maxDistanceTreshold*/) {
            nearVerticesIndexes.push_back(vertex.getIndex());
            nearVerticesDistances.push_back(distanceToNewVertex);
        }
    }
    return std::make_pair(nearVerticesIndexes, nearVerticesDistances);
}


void RrtStar::rewire(const util::Vertex &new_vertex, const std::vector<std::size_t> &nearVerticesIndexes, const std::vector<double> &distances)
{
    for (std::size_t indexOfIndexes = 0; indexOfIndexes < nearVerticesIndexes.size(); indexOfIndexes++) {

        std::size_t indexOfVertexInConsideration = nearVerticesIndexes.at(indexOfIndexes);
        double distanceToVertexInConsideration = distances.at(indexOfIndexes);
        util::Vertex &vertexInConsideration = vertexList_.at(indexOfVertexInConsideration);
        bool isCostSmallerViaNewNode =
            vertexInConsideration.getCost() > (new_vertex.getCost() + distanceToVertexInConsideration);
        bool colision = isCollision(new_vertex, vertexInConsideration);
        if (isCostSmallerViaNewNode && !colision) {
            vertexInConsideration.setParentIndex(new_vertex.getIndex());
            vertexInConsideration.setCost(distanceToVertexInConsideration + new_vertex.getCost());
        }
    }
}

double RrtStar::getDistance(util::Vertex startVertex, util::Vertex endVertex)
{
    return map_->worldDistanceEuclidean(startVertex.getLocation(), endVertex.getLocation());
}


void RrtStar::setGamma(double gamma)
{
    gamma_ = gamma;
}
void RrtStar::setRunToMaxIterations(bool setting)
{
    runTillMaxIterations_ = setting;
}
util::Vertex RrtStar::searchBestGoalNode()
{
    return util::Vertex(util::Point());
}
bool RrtStar::isCollision(const util::Vertex &from, const util::Vertex &to)
{

    double xFrom = from.getLocation().x;
    double yFrom = from.getLocation().y;
    double xTo = to.getLocation().x;
    double yTo = to.getLocation().y;

    // get the angle between the random point and our closest point (in rads)

    double distanceBetweenNodes = getDistance(from, to);
    double angleBetweenNodes = atan2(yTo - yFrom, xTo - xFrom);
    int maxNumberOfExpandSteps = floor(distanceBetweenNodes / map_->getResolution());
    for (int i = 0; i < maxNumberOfExpandSteps; i++) {
        double xIncrement = from.getX() + map_->getResolution() * cos(angleBetweenNodes) * i;
        double yIncrement = from.getY() + map_->getResolution() * sin(angleBetweenNodes) * i;
        if (isObstacle(util::Vertex{ util::Point{ xIncrement, yIncrement } })) {
            return true;
        }
    }
    return false;
}
void RrtStar::initialize(const util::GridMap<unsigned char> &map, const util::Options &options)
{
    (void)map;
    (void)options;
}

};// namespace rrt
