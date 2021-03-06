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
#include "Vertex.h"
enum class ObstacleType
{
    ObstacleFree,
    ObstacleLowestNonFree = 1,
    ObstaclePossiblyCircumscribed = 128,
    ObstacleInscribed = 253,
    ObstacleLethal = 254,

};

namespace rrt
{
    class RrtStar : IPlanner
    {
    public:
        explicit RrtStar(const util::GridMap<unsigned char> &costmap);
        std::vector<util::Point> makePlan(const util::Point& start,
                      const util::Point& goal) override;
        void initialize(const util::GridMap<unsigned char> &map, const util::Options &options) override;
        std::vector<util::Point>
        buildPlan(int goal_index);
        int findPath(const util::Vertex& start,const util::Vertex& goal);
        void setGamma(double gamma);
        void setRunToMaxIterations(bool setting);
    private:
        double getRandomExtendDistance() const;
        double getDistance(util::Point start_point, util::Point end_point);
        double getDistance(util::Vertex startVertex, util::Vertex endVertex);
        const util::Vertex& getClosestVertex(const util::Vertex &vertex);
        util::Vertex steer(const util::Vertex &from, const util::Vertex &to, double maxExtendDistance);
        std::pair<std::vector<std::size_t>, std::vector<double>>
        findNearVerticesIndexes(const util::Vertex &newVertex, double maxDistanceTreshold);
        void chooseNewParent(util::Vertex &newVertex, const std::vector<std::size_t> &nearVertcesIndxes,
                             const std::vector<double> &nearVerticesDistances);
        util::Vertex getRandomVertex();
        void rewire(const util::Vertex &new_vertex, const std::vector<std::size_t> &nearVerticesIndexes,
                    const std::vector<double> &distances);
        bool isObstacle(const util::Vertex &vertex);
        bool isOnCollisionPath(const util::Vertex &vertex);
        bool isCollision(const util::Vertex &from, const util::Vertex &to);
        bool reachedGoal(const util::Vertex &new_vertex);
        util::Vertex searchBestGoalNode();

      private:
        util::GridMap<unsigned char> obstacleMap_;
        std::vector<util::Vertex> vertex_list_;
        int max_iterations_ = 2000;
        bool runTillMaxIterations_ = false;
        double gamma_ = 1.0;
        double max_extend_distance_ = 8.0;
        double max_distance_threshold_ = 8.0;
        int current_iterations_ = 0;
        double goal_radius_ = 1.0;
        util::Point start_;
        util::Point goal_;

    };
} // namespace rrt_star
#endif // RRTSTAR_H
