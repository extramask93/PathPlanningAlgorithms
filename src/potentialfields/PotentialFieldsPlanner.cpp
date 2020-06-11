//
// Created by damian on 19.04.2020.
//

#include "PotentialFieldsPlanner.h"

namespace pf {
PotentialFieldsPlanner::PotentialFieldsPlanner(std::shared_ptr<util::GridMap<unsigned char>> map) : IPlanner(map),
                                                                                potentialMap_(std::vector<float>(map->getCellWidth() * map->getCellHeight()), map->getCellWidth(), map->getCellHeight())
{
}


float PotentialFieldsPlanner::calculateAttractivePotential(const util::Location &current, const util::Location &goal) const
{
    return 0.5 * ATTRACTIVE_POTENTIAL_GAIN * map_->distanceEuclidean(current, goal);
}

float PotentialFieldsPlanner::calculateRepulsivePotential(const util::Location &current) const
{
    auto closestObstacleLocation = map_->findClosestObstacle(current);
    if (closestObstacleLocation != boost::none) {
        auto distanceToObstacle = map_->distanceEuclidean(current, *closestObstacleLocation);
        if(distanceToObstacle >= 1.0) { //if cell is farther than robot radious then it is safe
            return 0.0;
        }
        if (distanceToObstacle < 0.1) {
            distanceToObstacle = 0.1;
        }
        return 0.5 * REPULSIVE_POTENTIAL_GAIN * std::pow(1.0 / distanceToObstacle, 2);
    } else {
        return 0.0;
    }
}


void PotentialFieldsPlanner::calculatePotentialField(const util::Location &goal)
{
    auto width = potentialMap_.getCellWidth();
    auto height = potentialMap_.getCellHeight();
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            auto attractivePotential = calculateAttractivePotential(util::Location{ x, y }, goal);
            auto repulsivePotential = calculateRepulsivePotential(util::Location{ x, y });
            auto totalPotential = attractivePotential + repulsivePotential;
            potentialMap_[{ x, y }] = totalPotential;
        }
    }
}

std::vector<util::Point> PotentialFieldsPlanner::makePlan(const util::Point &startP, const util::Point &goalP)
{
    util::Location start = map_->worldToMap(startP);
    util::Location goal = map_->worldToMap(goalP);

    calculatePotentialField(goal);
    start_ = start;
    goal_ = goal;
    std::vector<util::Point> path;
    path.push_back(startP);
    auto distanceToGoal = map_->distanceEuclidean(start, goal);
    auto motions = util::Robot::getMotionModel();
    auto currentLocation = start;
    int i = 0;
    double previousDistance = 0;
    while (distanceToGoal >= map_->getResolution()) {
        auto minPotential = std::numeric_limits<float>::infinity();
        auto minLocation = util::Location{};
        for (const auto &motion : motions) {
            float potential;
            util::Location nextLocation = currentLocation + motion;
            try {
                potential = potentialMap_[nextLocation];
            } catch (util::OutOfBoundsException) {
                continue;
            }
            if (potential <= minPotential) {
                minPotential = potential;
                minLocation = nextLocation;
            }
        }
        currentLocation = minLocation;
        distanceToGoal = map_->distanceEuclidean(minLocation, goal);
        if((i % 10) == 0) {
            if(distanceToGoal == previousDistance) {
                return std::vector<util::Point>{};
            }
            previousDistance = distanceToGoal;
        }
        path.push_back(map_->mapToWorld(minLocation));
        i++;
    }
    path.push_back(goalP);

    return path;
}



}// namespace pf
