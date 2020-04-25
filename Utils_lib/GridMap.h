//
// Created by damian on 25.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_GRIDMAP_H
#define POTENTIALFIELDSPROJECT_GRIDMAP_H

#include <vector>
#include <cmath>
#include <Location.h>
#include <Robot.h>
#include <MapException.h>
#include <boost/optional.hpp>
#include <boost/format.hpp>
#include <iomanip>

namespace util {
    template<typename CELL_T>
    class GridMap {
    public:
        explicit GridMap(std::vector<CELL_T> ogm, unsigned width, unsigned height, double resolution = 1.0);
        unsigned getCellWidth() const;
        unsigned getCellHeight() const;
        double getResolution() const;
        bool isWithinMapBounds(util::Location location) const;
        bool isFree(util::Location location) const noexcept(false);
        double distanceEuclidean(util::Location from, util::Location to) const noexcept(false);
        double distanceManhattan(util::Location from, util::Location to) const noexcept(false);

        std::vector<util::Location> findAllObstacles() const;

        boost::optional<util::Location> findClosestObstacle(util::Location location) const;

        util::Location indexToMap(unsigned index) const noexcept(false);

        CELL_T& operator[](const util::Location &);
        unsigned mapToIndex(util::Location location) const noexcept(false);
        void drawPath(std::ostream &out, std::vector<util::Location> path);
        template<typename T>
        friend std::ostream& operator<<(std::ostream &out, const util::GridMap<T> &gridMap);
    protected:
        void throwIfOutOfBounds(util::Location location) const;
    protected:
        std::vector<CELL_T> ogm_;
        unsigned mapWidth_;
        unsigned mapHeight_;
        double resolution_;
    };

    template<typename CELL_T>
    GridMap<CELL_T>::GridMap(std::vector<CELL_T> ogm, unsigned int width, unsigned int height, double resolution) {
        ogm_ = ogm;
        mapWidth_ = width;
        mapHeight_ = height;
        resolution_ = resolution;
    }

    template<typename CELL_T>
    bool GridMap<CELL_T>::isWithinMapBounds(util::Location location) const {
        if (location.x < 0 || static_cast<unsigned>(location.x) > mapWidth_
            || location.y < 0 || static_cast<unsigned>(location.y) > mapHeight_) {
            return false;
        } else {
            return true;
        }
    }
    template<typename CELL_T>
    bool GridMap<CELL_T>::isFree(util::Location location) const noexcept(false) {
        throwIfOutOfBounds(location);
        return ogm_[mapToIndex(location)] == 0;

    }
    template<typename CELL_T>
    unsigned GridMap<CELL_T>::mapToIndex(util::Location location) const noexcept(false) {
        throwIfOutOfBounds(location);
        return location.y * mapWidth_ + location.x;
    }

    template<typename CELL_T>
    void GridMap<CELL_T>::throwIfOutOfBounds(util::Location location) const {
        if (!isWithinMapBounds(location)) {
            auto formater = boost::format("Location: (%1%,%2%) is out of bounds") % location.x % location.y;
            throw util::OutOfBoundsException(formater.str());
        }

    }

    template<typename CELL_T>
    double GridMap<CELL_T>::distanceEuclidean(util::Location from, util::Location to) const noexcept(false) {
        throwIfOutOfBounds(from);
        throwIfOutOfBounds(to);
        double distanceX= std::pow(from.x - to.x, 2);
        double distanceY= std::pow(from.y - to.y, 2);
        return std::sqrt(distanceX + distanceY) * resolution_;
    }

    template<typename CELL_T>
    double GridMap<CELL_T>::distanceManhattan(util::Location from, util::Location to) const noexcept(false) {
       throwIfOutOfBounds(from);
       throwIfOutOfBounds(to);
       double distanceX = std::abs(from.x-to.x);
       double distanceY = std::abs(from.y- to.y);
       return (distanceY + distanceX)*resolution_;
    }

    template<typename CELL_T>
    util::Location GridMap<CELL_T>::indexToMap(unsigned int index) const noexcept(false) {
        int y = index / mapWidth_;
        int x = index - (y * mapWidth_);
        util::Location result = util::Location{x,y};
        throwIfOutOfBounds(result);
        return result;
    }

    template<typename CELL_T>
    std::vector<util::Location> GridMap<CELL_T>::findAllObstacles() const {
        std::vector<util::Location> obstacleIndexes;
        for(uint index =0; index < ogm_.size(); index++) {
            if(ogm_[index] == 1) {
                obstacleIndexes.push_back({indexToMap(index)});
            }
        }
        return obstacleIndexes;
    }

    template<typename CELL_T>
    boost::optional<util::Location> GridMap<CELL_T>::findClosestObstacle(util::Location location) const {
        boost::optional<util::Location> closest {};
        auto closestDistance = std::numeric_limits<double>::max();
        for(const auto &obstacle : findAllObstacles()) {
            if(distanceEuclidean(location, obstacle) <= closestDistance) {
                closestDistance = distanceEuclidean(location,obstacle);
                closest = obstacle;
            }
        }
        return closest;
    }

    template<typename CELL_T>
    CELL_T& GridMap<CELL_T>::operator[](const util::Location &location) {
        return ogm_[mapToIndex(location)];
    }
    template<typename T>
    std::ostream &operator<<(std::ostream &out, const GridMap<T> &gridMap) {
        out<<std::fixed<<std::setprecision(1);
        for(uint index = 0; index < gridMap.ogm_.size(); index++) {
            if(!(index % gridMap.mapWidth_)) {
                out <<"\n";
            }
            out<<std::left<<std::setw(4)<<gridMap.ogm_[index]<<" | ";

        }
        return out;
    }

    template<typename CELL_T>
    unsigned GridMap<CELL_T>::getCellWidth() const{
        return mapWidth_;
    }

    template<typename CELL_T>
    unsigned GridMap<CELL_T>::getCellHeight() const{
        return mapHeight_;
    }

    template<typename CELL_T>
    double GridMap<CELL_T>::getResolution() const{
        return resolution_;
    }

    template<typename CELL_T>
    void GridMap<CELL_T>::drawPath(std::ostream &out, std::vector<util::Location> path) {
        out<<std::fixed<<std::setprecision(1);
        auto motionModel = util::Robot::getMotionModel();
        auto motionModelArrows = util::Robot::getMotionModelArrows();
        for(uint index = 0; index < ogm_.size(); index++) {
            if(!(index % mapWidth_)) {
                out <<"\n";
            }
            auto iterator = std::find_if(path.begin(), path.end(), [&](const auto &location){ return this->indexToMap(index) == location;});
            if(iterator != path.end() && (iterator+1) != path.end()) {
                auto movementLoc =  *(iterator+1) - indexToMap(index);
                auto iter2 = std::find_if(motionModel.begin(), motionModel.end(),
                        [&](const auto &location){ return movementLoc == location;});
                auto distance = std::distance(motionModel.begin(),iter2);
                out << std::left << std::setw(4) << util::Robot::getMotionModelArrows()[distance]<< " | ";
            } else {
                out << std::left << std::setw(4) << ogm_[index] << " | ";
            }
        }
    }

}


#endif //POTENTIALFIELDSPROJECT_GRIDMAP_H
