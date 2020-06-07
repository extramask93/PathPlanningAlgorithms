//
// Created by damian on 07.06.2020.
//

#ifndef PLANNING_SAMPLINGBASE_H
#define PLANNING_SAMPLINGBASE_H
#include <Vertex.h>
#include <GridMap.h>
#include <random>
#include <deque>
namespace sb {
/*using CRTP to access map property*/
template<class T>
class SamplingBase
{
  protected:
    [[nodiscard]] util::Vertex getRandomVertex() const;
    [[nodiscard]] double getRandomExtendDistance() const;
    [[nodiscard]] bool reachedGoal(const util::Vertex &vertex) const;
    [[nodiscard]] bool isObstacle(const util::Point &point) const;
    [[nodiscard]] bool isObstacle(const util::Vertex &point) const;
    [[nodiscard]] bool isOnCollisionPath(const util::Vertex &vertex) const;
    [[nodiscard]] util::Vertex findClosestVertex(const util::Vertex &from) const;
    [[nodiscard]] util::Vertex steer(const util::Vertex &from, const util::Vertex &to) const;
    [[nodiscard]] std::vector<util::Point> buildPlan(int goalIndex) const;
  private:
    constexpr const T& derived() const { return *static_cast<const T*>(this); }
    constexpr T& derived()  { return *static_cast<T*>(this); }
};
template<class T>
util::Vertex SamplingBase<T>::getRandomVertex() const
{
    static std::mt19937 randomGenerator(std::random_device{}());
    using Distribution = std::uniform_real_distribution<double>;
    static Distribution xDistribution(derived().map_->getOriginX(),
        derived().map_->getOriginX() + derived().map_->getWorldWidth());
    static Distribution yDistribution(derived().map_->getOriginY(),
        derived().map_->getOriginY() + derived().map_->getWorldHeight());
    static Distribution goalSamplingDistribution = std::uniform_real_distribution<double>(0, 1);
    // roll goal with 5%-10% probability -> https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf
    if (goalSamplingDistribution(randomGenerator) >= (1 - derived().goalSamplingRatio_)) {
        return util::Vertex(derived().goalVertex_.getLocation(), util::Vertex::NO_ID, util::Vertex::NO_PARENT);
    }
    return util::Vertex{ util::Point(xDistribution(randomGenerator), yDistribution(randomGenerator)), util::Vertex::NO_ID, util::Vertex::NO_PARENT };
}
template<class T>
double SamplingBase<T>::getRandomExtendDistance() const
{
    static std::mt19937 randomGenerator(std::random_device{}());
    using Distribution = std::uniform_real_distribution<double>;
    static Distribution distribution(derived().map_->getResolution()*derived().map_->getCellWidth()*0.3,
                                     derived().map_->getCellWidth() * derived().map_->getResolution() * 0.5);
    return distribution(randomGenerator);
}
template<class T>
bool SamplingBase<T>::reachedGoal(const util::Vertex &vertex) const
{
    double distance = derived().map_->worldDistanceEuclidean(vertex.getLocation(), derived().goalVertex_.getLocation());
    return distance <= derived().goalRadius_;
}
template<class T>
bool SamplingBase<T>::isObstacle(const util::Point &point) const
{

    auto mapLocation = derived().map_->worldToMap(point);
    return !derived().map_->isFree(mapLocation);
}
template<class T>
bool SamplingBase<T>::isOnCollisionPath(const util::Vertex &vertex) const
{
    for (unsigned i = 0; i < vertex.getPath().size(); i++) {
        auto mapLocation = derived().map_->worldToMap(vertex.getPath().at(i));
        if (!derived().map_->isFree(mapLocation)) {
            return true;
        }
    }
    return false;
}
template<class T>
util::Vertex SamplingBase<T>::findClosestVertex(const util::Vertex &from) const
{

    double closestDistance = std::numeric_limits<double>::infinity();
    util::Vertex closestVertex = derived().vertexList_.at(0);
    for (const util::Vertex &vertex : derived().vertexList_) {
        double distanceToNewVertex = derived().map_->worldDistanceEuclidean(from.getLocation(), vertex.getLocation());
        if (distanceToNewVertex < closestDistance) {
            closestDistance = distanceToNewVertex;
            closestVertex = vertex;
        }
    }

    return derived().vertexList_.at(closestVertex.getIndex());
}
template<class T>
util::Vertex SamplingBase<T>::steer(const util::Vertex &from, const util::Vertex &to) const
{
    double xFrom = from.getLocation().x;
    double yFrom = from.getLocation().y;
    double xTo = to.getLocation().x;
    double yTo = to.getLocation().y;
    // get the angle between the random point and our closest point (in rads)

    double distanceBetweenNodes = derived().map_->worldDistanceEuclidean(from.getLocation(), to.getLocation());
    double maxExtend = getRandomExtendDistance();
    if (distanceBetweenNodes > maxExtend) {
        distanceBetweenNodes = maxExtend;
    }

    util::Vertex newVertex(from.getLocation(), derived().vertexList_.size(), from.getIndex());
    newVertex.addToPath(from.getLocation());
    double angleBetweenNodes = atan2(yTo - yFrom, xTo - xFrom);
    int maxNumberOfExpandSteps = floor(distanceBetweenNodes / derived().map_->getResolution());
    for (int i = 0; i < maxNumberOfExpandSteps; i++) {
        double xIncrement = newVertex.getX() + derived().map_->getResolution() * cos(angleBetweenNodes);
        double yIncrement = newVertex.getY() + derived().map_->getResolution() * sin(angleBetweenNodes);
        auto newPoint = util::Point{ xIncrement, yIncrement };
        if (isObstacle(newPoint)) {
            break;
        }
        newVertex.setLocation(newPoint);
        newVertex.addToPath(newPoint);
    }
    distanceBetweenNodes = derived().map_->worldDistanceEuclidean(newVertex.getLocation(), to.getLocation());
    if (distanceBetweenNodes <= derived().map_->getResolution()) {
        newVertex.addToPath(to.getLocation());
    }

    return newVertex;
}
template<class T>
std::vector<util::Point> SamplingBase<T>::buildPlan(int goalIndex) const
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
        current_index = derived().vertexList_.at(current_index).getParentIndex();
    }
    indexPath.push_front(INDEX_OF_START_VERTEX);

    for (int i : indexPath) {
        util::Point pos = derived().vertexList_[i].getLocation();
        plan.push_back(pos);
    }
    return plan;
}
template<class T>
bool SamplingBase<T>::isObstacle(const util::Vertex &point) const
{
    return isObstacle(point.getLocation());
}
}// namespace sb


#endif//PLANNING_SAMPLINGBASE_H
