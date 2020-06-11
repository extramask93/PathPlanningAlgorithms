#ifndef RRTSTAR_H
#define RRTSTAR_H
#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <boost/optional.hpp>
#include <GridMap.h>
#include <IPlanner.h>
#include <Vertex.h>
#include <SamplingBase.h>
enum class ObstacleType {
    ObstacleFree,
    ObstacleLowestNonFree = 1,
    ObstaclePossiblyCircumscribed = 128,
    ObstacleInscribed = 253,
    ObstacleLethal = 254,

};

namespace rrt {
class RrtStar : public IPlanner
    , public sb::SamplingBase<RrtStar>
{
    friend class SamplingBase<RrtStar>;

  public:
    explicit RrtStar(std::shared_ptr<util::GridMap<unsigned char>> map);
    std::vector<util::Point> makePlan(const util::Point &start,
        const util::Point &goal) override;
    int findPath(const util::Vertex &start, const util::Vertex &goal);
    void setGamma(double gamma);
    void setRunToMaxIterations(bool setting);

  private:
    double getDistance(util::Vertex startVertex, util::Vertex endVertex);
    std::pair<std::vector<std::size_t>, std::vector<double>>
        findNearVerticesIndexes(const util::Vertex &newVertex, double maxDistanceTreshold);
    void chooseNewParent(util::Vertex &newVertex, const std::vector<std::size_t> &nearVertcesIndxes, const std::vector<double> &nearVerticesDistances);
    void rewire(const util::Vertex &new_vertex, const std::vector<std::size_t> &nearVerticesIndexes, const std::vector<double> &distances);
    bool isCollision(const util::Vertex &from, const util::Vertex &to);
    util::Vertex searchBestGoalNode();

  private:
    util::Vertex goalVertex_;
    util::Vertex startVertex_;
    std::vector<util::Vertex> vertexList_;
    int max_iterations_ = 3500;
    bool runTillMaxIterations_ = false;
    double gamma_ = 1.0;
    double max_extend_distance_ = 8.0;
    double goalSamplingRatio_ = 0.1;
    double max_distance_threshold_ = 8.0;
    int current_iterations_ = 0;
    double goalRadius_ = 1.0;
    util::Point start_;
    util::Point goal_;
};
}// namespace rrt
#endif// RRTSTAR_H
