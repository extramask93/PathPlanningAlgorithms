//
// Created by damian on 19.04.2020.
//

#include "PotentialFieldsPlanner.h"
#include <cmath>
#include <iostream>
#include <iomanip>

namespace pf {
    PotentialFieldsPlanner::PotentialFieldsPlanner(const std::vector<bool> &ogm, unsigned int width,
                                                   unsigned int height, double resolution) {
        ogm_ = ogm;
        potentialMap_ = std::vector<double> (ogm.size(), 0.0);
        mapWidth_ = width;
        mapHeight_ = height;
        mapResolution_ = resolution;
        findObstacles();
    }

    double PotentialFieldsPlanner::calculateAttractivePotential(uint currentCelIndex, uint goalIndex, double KD) {
        return 0.5 * ATTRACTIVE_POTENTIAL_GAIN * distance(currentCelIndex,goalIndex);
    }

    double PotentialFieldsPlanner::distance(uint fromIndex, uint toIndex) {
        auto current = indexToMap(fromIndex);
        auto goal = indexToMap(toIndex);
        double distanceX= std::pow(current->first - goal->first, 2);
        double distanceY= std::pow(current->second - goal->second, 2);
        return std::sqrt(distanceX + distanceY);
    }

    boost::optional<std::pair<int,int>> PotentialFieldsPlanner::indexToMap(unsigned int index) {
        int y = index / mapWidth_;
        int x = index - (y * mapWidth_);
        return boost::optional<std::pair<int, int>>(std::make_pair(x,y));
    }

    double PotentialFieldsPlanner::calculateRepulsivePotential(uint currentIndex) {
        auto closestObstacleIndex = findClosestObstacleIndex(currentIndex);
        if(closestObstacleIndex) {
            auto distanceToObstacle = distance(currentIndex, *closestObstacleIndex);
            /*prevent 0 in denomitnator*/
            if(distanceToObstacle < 0.1) {
                distanceToObstacle = 0.1;
            }
            return 0.5 * REPULSIVE_POTENTIAL_GAIN * std::pow(1.0 /  distanceToObstacle ,2);
        } else {
            return 0.0;
        }
    }

    void PotentialFieldsPlanner::findObstacles() {
        obstacleIndexes_.clear();
        for(uint index =0; index < ogm_.size(); index++) {
            if(ogm_[index] == OBSTACLE) {
                obstacleIndexes_.push_back(index);
            }
        }

    }

    boost::optional<uint> PotentialFieldsPlanner::findClosestObstacleIndex(uint index) {
        boost::optional<uint> closestId {};
        auto closestDistance = std::numeric_limits<double>::max();
        for(auto obstacleIndex : obstacleIndexes_) {
            if(distance(index, obstacleIndex) <= closestDistance) {
                closestDistance = distance(index,obstacleIndex);
                closestId = obstacleIndex;
            }
        }
        return closestId;
    }

    void PotentialFieldsPlanner::calculatePotentialField(uint goalIndex) {
        for(uint currentIndex =0; currentIndex < potentialMap_.size(); currentIndex++) {
            auto attractivePotential = calculateAttractivePotential(currentIndex, goalIndex);
            auto repulsivePotential = calculateRepulsivePotential(currentIndex);
            auto totalPotential = attractivePotential + repulsivePotential;
            potentialMap_[currentIndex] = totalPotential;
        }

    }

    void PotentialFieldsPlanner::printMapOfPotentials() {
        std::cout<<std::fixed<<std::setprecision(1);
        for(uint index = 0; index < potentialMap_.size(); index++) {

            if(!(index % mapWidth_)) {
                std::cout <<"\n";
            }
            std::cout<<std::left<<std::setw(6)<<potentialMap_[index]<<" | ";

        }
    }

    std::vector<uint> PotentialFieldsPlanner::makePlan(uint startIndex, uint goalIndex) {
        calculatePotentialField(goalIndex);
        startIndex_ = startIndex;
        goalIndex_ = goalIndex;
        std::vector<uint> path;
        movePath_.clear();
        path.push_back(startIndex);
        auto distanceToGoal = distance(startIndex, goalIndex);
        auto motions = getMotionModel();
        auto currentLocation = *indexToMap(startIndex);
        while(distanceToGoal >= mapResolution_) {
            auto minPotential = std::numeric_limits<double>::infinity();
            auto minIndex = std::numeric_limits<uint>::max();
            for(const auto& motion : motions) {
                double potential = std::numeric_limits<double>::infinity();
                std::pair<int,int> nextLocation = currentLocation;
                nextLocation.first = currentLocation.first + motion.first;
                nextLocation.second = currentLocation.second + motion.second;
                if(!isWithinMapBounds(nextLocation)) {
                    continue;
                }
                potential = potentialMap_[mapToIndex(nextLocation)];
                if(potential <= minPotential) {
                    minPotential = potential;
                    minIndex = mapToIndex(nextLocation);
                }
            }
            movePath_.push_back({mapToIndex(currentLocation),currentLocation.second != 0? '|':'-'});
            currentLocation = *indexToMap(minIndex);
            distanceToGoal = distance(minIndex,goalIndex);
            path.push_back(minIndex);
        }
        movePath_.push_back({goalIndex,indexToMap(path.back())->second != 0 ? '|':'-'});
        path.push_back(goalIndex);
        return path;
    }

    std::vector<std::pair<int, int>> PotentialFieldsPlanner::getMotionModel() const {
        return std::vector<std::pair<int, int>>({
            {0, -1}, /*|*/
            {-1, 0}, /*-*/
            {0,1}, /*|*/
            {1,0} /*-*/
        });
    }

    uint PotentialFieldsPlanner::mapToIndex(std::pair<int,int> location) {
        return location.second * mapWidth_ + location.first;
    }

    bool PotentialFieldsPlanner::isWithinMapBounds(std::pair<int,int> location) {
        if(location.first < 0  || location.first > mapWidth_
            || location.second < 0 || location.second > mapHeight_) {
            return false;
        } else {
            return true;
        }
    }

    void PotentialFieldsPlanner::printMap() {
        std::cout<<std::fixed<<std::setprecision(1);
        for(uint index = 0; index < ogm_.size(); index++) {

            if(!(index % mapWidth_)) {
                std::cout <<"\n";
            }
            std::cout<<std::left<<std::setw(4)<<ogm_[index]<<" | ";

        }
    }

    void PotentialFieldsPlanner::printPath() {
        std::cout<<std::fixed<<std::setprecision(1);
        for(uint index = 0; index < ogm_.size(); index++) {

            if(!(index % mapWidth_)) {
                std::cout <<"\n";
            }
            auto it = std::find_if(movePath_.begin(), movePath_.end(), [=](const auto &item) { return item.first == index;});
            if(it != movePath_.end()) {
                if(it->first == goalIndex_) {
                    it->second = '*';
                }
                if(it->first == startIndex_) {
                    it->second = '+';
                }
                std::cout << std::left << std::setw(4) << it->second;
            } else {

                std::cout << std::left << std::setw(4) << ogm_[index];
            }

        }
    }
}
