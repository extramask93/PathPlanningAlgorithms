//
// Created by damian on 19.04.2020.
//

#include "PotentialFieldsPlanner.h"

namespace pf {
PotentialFieldsPlanner::PotentialFieldsPlanner(const util::GridMap<unsigned char> &ogm) : ogm_(ogm),
                                                                                potentialMap_(std::vector<double>(ogm.getCellWidth() * ogm.getCellHeight()), ogm.getCellWidth(), ogm.getCellHeight())
{
}


double PotentialFieldsPlanner::calculateAttractivePotential(const util::Location &current, const util::Location &goal) const
{
    return 0.5 * ATTRACTIVE_POTENTIAL_GAIN * ogm_.distanceEuclidean(current, goal);
}

double PotentialFieldsPlanner::calculateRepulsivePotential(const util::Location &current) const
{
    auto closestObstacleLocation = ogm_.findClosestObstacle(current);
    if (closestObstacleLocation != boost::none) {
        auto distanceToObstacle = ogm_.distanceEuclidean(current, *closestObstacleLocation);
        if(distanceToObstacle >= 0.5) { //if cell is farther than robot radious then it is safe
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
    util::Location start = ogm_.worldToMap(startP);
    util::Location goal = ogm_.worldToMap(goalP);

    calculatePotentialField(goal);
    start_ = start;
    goal_ = goal;
    std::vector<util::Point> path;
    path.push_back(startP);
    auto distanceToGoal = ogm_.distanceEuclidean(start, goal);
    auto motions = util::Robot::getMotionModel();
    auto currentLocation = start;
    while (distanceToGoal >= ogm_.getResolution()) {
        auto minPotential = std::numeric_limits<double>::infinity();
        auto minLocation = util::Location{};
        for (const auto &motion : motions) {
            double potential;
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
        distanceToGoal = ogm_.distanceEuclidean(minLocation, goal);
        path.push_back(ogm_.mapToWorld(minLocation));
    }
    path.push_back(goalP);

    return path;
}


}// namespace pf
