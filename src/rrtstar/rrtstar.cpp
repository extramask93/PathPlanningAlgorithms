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
RrtStar::RrtStar(const util::GridMap<unsigned char> &costmap)
    : obstacleMap_(costmap)
{
    gamma_ = 5;
    max_extend_distance_ = obstacleMap_.getResolution() * 3;
}


std::vector<util::Point> RrtStar::makePlan(const util::Point &start,
    const util::Point &goal)
{
    current_iterations_ = 0;
    vertex_list_.clear();
    util::Vertex startVertex(start);
    util::Vertex goalVertex(goal);
    start_ = start;
    goal_ = goal;
    int goal_index = RrtStar::findPath(startVertex, goalVertex);
    auto plan = RrtStar::buildPlan(goal_index);
    return plan;
}

util::Vertex RrtStar::getRandomVertex()
{
    static std::mt19937 randomGenerator(10/*std::random_device{}()*/);
    static unsigned callCounter = 0;
    using Distribution = std::uniform_real_distribution<double>;
    static Distribution xDistribution(obstacleMap_.getOriginX(),
        obstacleMap_.getOriginX() + obstacleMap_.getWorldWidth());
    static Distribution yDistribution(obstacleMap_.getOriginY(),
        obstacleMap_.getOriginY() + obstacleMap_.getWorldHeight());
    // roll goal with 5%-10% probability -> https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf
    if(callCounter % 10) {
        callCounter++;
        return util::Vertex{goal_, util::Vertex::NO_ID, util::Vertex::NO_PARENT};
    }
    return util::Vertex{ util::Point(xDistribution(randomGenerator), yDistribution(randomGenerator)), util::Vertex::NO_ID, util::Vertex::NO_PARENT };
}
util::Vertex RrtStar::steer(const util::Vertex &from, const util::Vertex &to, double maxExtendDistance)
{
    double xFrom = from.getLocation().x;
    double yFrom = from.getLocation().y;
    double xTo = to.getLocation().x;
    double yTo = to.getLocation().y;

    // get the angle between the random point and our closest point (in rads)

    double distanceBetweenNodes = getDistance(from, to);
    double maxExtend = getRandomExtendDistance();
    if (distanceBetweenNodes > maxExtend) {
        distanceBetweenNodes = maxExtend;
    }

    util::Vertex new_vertex(from.getLocation(), vertex_list_.size(), from.getIndex());//
    new_vertex.addToPath(from.getLocation());
    double angleBetweenNodes = atan2(yTo - yFrom, xTo - xFrom);
    int maxNumberOfExpandSteps = floor(distanceBetweenNodes / obstacleMap_.getResolution());
    for (int i = 0; i < maxNumberOfExpandSteps; i++) {
        double xIncrement = new_vertex.getX() + obstacleMap_.getResolution() * cos(angleBetweenNodes);
        double yIncrement = new_vertex.getY() + obstacleMap_.getResolution() * sin(angleBetweenNodes);
        if (isObstacle(util::Vertex{ util::Point{ xIncrement, yIncrement } })) {
            break;
        }
        new_vertex.setX(xIncrement);
        new_vertex.setY(yIncrement);
        new_vertex.addToPath(new_vertex.getLocation());
    }
    distanceBetweenNodes = getDistance(new_vertex, to);
    if (distanceBetweenNodes <= obstacleMap_.getResolution()) {
        new_vertex.addToPath({ to.getX(), to.getY() });
    }

    return new_vertex;
}


int RrtStar::findPath(const util::Vertex &start, const util::Vertex &goal)
{
    int goal_index = -1;
    current_iterations_ = 0;
    vertex_list_.clear();

    if (isObstacle(start) || isObstacle(goal)) {
        return goal_index;
    }
    util::Vertex root(start.getLocation(), 0, util::Vertex::NO_PARENT);
    vertex_list_.push_back(root);
    while (current_iterations_ < max_iterations_) {
        util::Vertex randomVertex = RrtStar::getRandomVertex();
        if (isObstacle(randomVertex)) {
            continue;
        }
        const util::Vertex &closest_vertex = getClosestVertex(randomVertex);
        util::Vertex newVertex = steer(closest_vertex, randomVertex, max_extend_distance_);
        newVertex.setParentIndex(closest_vertex.getIndex());
        if (isOnCollisionPath(newVertex)) {
            continue;
        }
        auto nearVerticesAndDistances = findNearVerticesIndexes(newVertex, max_distance_threshold_);
        std::vector<std::size_t> &nearVerticesIndices = nearVerticesAndDistances.first;
        std::vector<double> &nearDistances = nearVerticesAndDistances.second;
        chooseNewParent(newVertex, nearVerticesIndices, nearDistances);
        vertex_list_.push_back(newVertex);
        rewire(newVertex, nearVerticesIndices, nearDistances);
        if (!runTillMaxIterations_ && reachedGoal(newVertex)) {

            util::Vertex goalVertex{ goal.getLocation(), static_cast<int>(vertex_list_.size()), newVertex.getIndex() };
            vertex_list_.push_back(goalVertex);
            goal_index = goalVertex.getIndex();
            break;
        }
        current_iterations_++;
    }
    if(runTillMaxIterations_) {
        auto closestVertex = getClosestVertex(goal);
        auto goalVertex = goal;
        goalVertex.setParentIndex(closestVertex.getIndex());
        goalVertex.setIndex(vertex_list_.size());
        vertex_list_.push_back(goalVertex);
        goal_index = goalVertex.getIndex();
        //find best node to connect to the goal
    }
    return goal_index;
}

void RrtStar::chooseNewParent(util::Vertex &newVertex, const std::vector<std::size_t> &nearVertcesIndxes, const std::vector<double> &nearVerticesDistances)
{
    if(nearVertcesIndxes.empty()) {
        auto parent = vertex_list_[newVertex.getParentIndex()];
        newVertex.setCost(parent.getCost() + getDistance(newVertex,parent));
        return;
    }
    auto iteratorToMinDistance = std::min_element(nearVerticesDistances.begin(), nearVerticesDistances.end());
    std::size_t indexOfMinDistance = std::distance(nearVerticesDistances.begin(), iteratorToMinDistance);
    assert(("Near vertex was not found, probably extending farther than parent search distance", nearVertcesIndxes.size() > indexOfMinDistance));
    std::size_t indexOfClosestvertex = nearVertcesIndxes.at(indexOfMinDistance);
    util::Vertex &closestVertex = vertex_list_.at(indexOfClosestvertex);
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
    double n = vertex_list_.size();
    double intermediate = std::pow(std::log(n) / n,0.5);
    double radius = gamma_ * intermediate;
    std::vector<std::size_t> nearVerticesIndexes{};
    std::vector<double> nearVerticesDistances{};
    for (const util::Vertex &vertex : vertex_list_) {
        double distanceToNewVertex = getDistance(newVertex.getLocation(), vertex.getLocation());
        if (distanceToNewVertex <= radius/*maxDistanceTreshold*/) {
            nearVerticesIndexes.push_back(vertex.getIndex());
            nearVerticesDistances.push_back(distanceToNewVertex);
        }
    }
    return std::make_pair(nearVerticesIndexes, nearVerticesDistances);
}
const util::Vertex &RrtStar::getClosestVertex(const util::Vertex &newVertex)
{

    double closestDistance = std::numeric_limits<double>::infinity();
    util::Vertex closestVertex = vertex_list_.at(0);

    for (util::Vertex &vertex : vertex_list_) {
        double distanceToNewVertex = getDistance(vertex.getLocation(), newVertex.getLocation());

        if (distanceToNewVertex < closestDistance) {
            closestDistance = distanceToNewVertex;
            closestVertex = vertex;
        }
    }

    return vertex_list_.at(closestVertex.getIndex());
}
double RrtStar::getDistance(util::Point start_point, util::Point end_point)
{
    double x1 = start_point.x;
    double y1 = start_point.y;

    double x2 = end_point.x;
    double y2 = end_point.y;

    double distanceSquared = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));

    return distanceSquared;
}

bool RrtStar::isOnCollisionPath(const util::Vertex &vertex)
{
    for (std::size_t i = 0; i < vertex.getPath().size(); i++) {
        auto mapLocation = obstacleMap_.worldToMap(vertex.getPath().at(i));
        if (obstacleMap_[mapLocation] == OBSTACLE) {
            return true;
        }
    }
    return false;
}

bool RrtStar::isObstacle(const util::Vertex &vertex)
{
    auto mapLocation = obstacleMap_.worldToMap(vertex.getLocation());
    return obstacleMap_[mapLocation] == OBSTACLE;
}

void RrtStar::rewire(const util::Vertex &new_vertex, const std::vector<std::size_t> &nearVerticesIndexes, const std::vector<double> &distances)
{
    for (std::size_t indexOfIndexes = 0; indexOfIndexes < nearVerticesIndexes.size(); indexOfIndexes++) {

        std::size_t indexOfVertexInConsideration = nearVerticesIndexes.at(indexOfIndexes);
        double distanceToVertexInConsideration = distances.at(indexOfIndexes);
        util::Vertex &vertexInConsideration = vertex_list_.at(indexOfVertexInConsideration);
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
    return getDistance(startVertex.getLocation(), endVertex.getLocation());
}

bool RrtStar::reachedGoal(const util::Vertex &new_vertex)
{


    // Check distance between current point and goal, if distance is less
    // than goal_radius_ return true, otherwise return false
    double distance = obstacleMap_.worldDistanceEuclidean(new_vertex.getLocation(), goal_);
    if (distance <= goal_radius_)
        return true;
    else
        return false;
}

std::vector<util::Point>
    RrtStar::buildPlan(int goal_index)
{
    // The plan we'll be adding to and returning
    std::vector<util::Point> plan;

    // no plan found
    if (goal_index == -1)
        return plan;

    // The list of vertex indices we pass through to get to the goal
    std::deque<int> index_path;
    int current_index = goal_index;
    const int INDEX_OF_START_VERTEX = 0;

    while (current_index > INDEX_OF_START_VERTEX) {
        index_path.push_front(current_index);
        current_index = vertex_list_.at(current_index).getParentIndex();
    }
    index_path.push_front(INDEX_OF_START_VERTEX);

    for (int i : index_path) {
        util::Point pos = util::Point{ vertex_list_[i].getX(), vertex_list_[i].getY() };
        plan.push_back(pos);
    }
    return plan;
}
double RrtStar::getRandomExtendDistance() const
{
    static std::mt19937 randomGenerator(10/*std::random_device{}()*/);
    using Distribution = std::uniform_real_distribution<double>;
    static Distribution distribution(obstacleMap_.getResolution(),
                                     obstacleMap_.getCellWidth() * obstacleMap_.getResolution() * 0.1);
    return distribution(randomGenerator);
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
    int maxNumberOfExpandSteps = floor(distanceBetweenNodes / obstacleMap_.getResolution());
    for (int i = 0; i < maxNumberOfExpandSteps; i++) {
        double xIncrement = from.getX() + obstacleMap_.getResolution() * cos(angleBetweenNodes)*i;
        double yIncrement = from.getY()+ obstacleMap_.getResolution() * sin(angleBetweenNodes)*i;
        if (isObstacle(util::Vertex{ util::Point{ xIncrement, yIncrement } })) {
            return true;
        }
    }
    return false;
}

};// namespace rrt
