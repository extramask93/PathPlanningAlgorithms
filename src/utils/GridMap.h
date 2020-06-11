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
#include "Point.h"
#ifdef MATPLOTLIB
#include <matplotlibcpp.h>
#endif
namespace util {
template<typename CELL_T>
class GridMap
{
public:
  explicit GridMap(const std::vector<CELL_T> &ogm, unsigned width, unsigned height, double resolution = 1.0);
  unsigned getCellWidth() const;
  unsigned getCellHeight() const;
  double getResolution() const;
  std::string getName() const;
  void setName(const std::string &name);
  bool isWithinMapBounds(util::Location location) const;
  bool isFree(util::Location location) const noexcept(false);
  double distanceEuclidean(util::Location from, util::Location to) const noexcept(false);
  double distanceManhattan(util::Location from, util::Location to) const noexcept(false);
  std::vector<util::Location> findAllObstacles() const;
  boost::optional<util::Location> findClosestObstacle(util::Location location) const;
  util::Location indexToMap(unsigned index) const noexcept(false);
  CELL_T &operator[](const util::Location &);
  const CELL_T &operator[](const util::Location &) const;
  unsigned mapToIndex(util::Location location) const noexcept(false);
  void plotMap();
  void plotPathOnMap(const std::vector<util::Point> &path);
  void drawPath(std::ostream &out, std::vector<util::Location> path);
  template<typename T>
  friend std::ostream &operator<<(std::ostream &out, const util::GridMap<T> &gridMap);
  /*World coordinate interface*/
  double getWorldWidth() const;
  util::Point mapToWorld(const util::Location &location) const;
  double getWorldHeight() const;
  double getOriginX() const;
  double getOriginY() const;
  util::Location worldToMap(const util::Point &point) const noexcept(false);
  double worldDistanceEuclidean(const util::Point &from, const util::Point &to) const;
  double worldDistanceManhattan(const util::Point &from, const util::Point &to) const;

protected:
  void throwIfOutOfBounds(util::Location location) const;

protected:
  std::vector<CELL_T> ogm_;
  unsigned mapWidth_;
  unsigned mapHeight_;
  double resolution_;
  util::Point origin_;
  std::string name_ = "Unnamed map";
};

template<typename CELL_T>
GridMap<CELL_T>::GridMap(const std::vector<CELL_T> &ogm, unsigned int width, unsigned int height, double resolution)
{
  ogm_ = ogm;
  mapWidth_ = width;
  mapHeight_ = height;
  resolution_ = resolution;
  origin_ = util::Point(0.0, 0.0);
}

template<typename CELL_T>
bool GridMap<CELL_T>::isWithinMapBounds(util::Location location) const
{
  if (location.x < 0 || static_cast<unsigned>(location.x) >= mapWidth_
      || location.y < 0 || static_cast<unsigned>(location.y) >= mapHeight_) {
    return false;
  } else {
    return true;
  }
}
template<typename CELL_T>
bool GridMap<CELL_T>::isFree(util::Location location) const noexcept(false)
{
  throwIfOutOfBounds(location);
  return ogm_[mapToIndex(location)] == 1;
}
template<typename CELL_T>
unsigned GridMap<CELL_T>::mapToIndex(util::Location location) const noexcept(false)
{
  throwIfOutOfBounds(location);
  return static_cast<unsigned>(location.y) * mapWidth_ + static_cast<unsigned>(location.x);
}

template<typename CELL_T>
void GridMap<CELL_T>::throwIfOutOfBounds(util::Location location) const
{
  if (!isWithinMapBounds(location)) {
    auto formater = boost::format("Location: (%1%,%2%) is out of bounds") % location.x % location.y;
    throw util::OutOfBoundsException(formater.str());
  }
}

template<typename CELL_T>
double GridMap<CELL_T>::distanceEuclidean(util::Location from, util::Location to) const noexcept(false)
{
  throwIfOutOfBounds(from);
  throwIfOutOfBounds(to);
  double distanceX = std::pow(from.x - to.x, 2);
  double distanceY = std::pow(from.y - to.y, 2);
  return std::sqrt(distanceX + distanceY) * resolution_;
}

template<typename CELL_T>
double GridMap<CELL_T>::distanceManhattan(util::Location from, util::Location to) const noexcept(false)
{
  throwIfOutOfBounds(from);
  throwIfOutOfBounds(to);
  double distanceX = std::abs(from.x - to.x);
  double distanceY = std::abs(from.y - to.y);
  return (distanceY + distanceX) * resolution_;
}

template<typename CELL_T>
util::Location GridMap<CELL_T>::indexToMap(unsigned int index) const noexcept(false)
{
  int y = static_cast<int>(index) / static_cast<int>(mapWidth_);
  int x = static_cast<int>(index) - (y * static_cast<int>(mapWidth_));
  util::Location result = util::Location{ x, y };
  throwIfOutOfBounds(result);
  return result;
}

template<typename CELL_T>
std::vector<util::Location> GridMap<CELL_T>::findAllObstacles() const
{
  std::vector<util::Location> obstacleIndexes;
  for (uint index = 0; index < ogm_.size(); index++) {
    if (ogm_[index] == 0) {
      obstacleIndexes.push_back({ indexToMap(index) });
    }
  }
  return obstacleIndexes;
}

template<typename CELL_T>
boost::optional<util::Location> GridMap<CELL_T>::findClosestObstacle(util::Location location) const
{
  boost::optional<util::Location> closest{};
  auto closestDistance = std::numeric_limits<double>::max();
  for (const auto &obstacle : findAllObstacles()) {
    if (distanceEuclidean(location, obstacle) <= closestDistance) {
      closestDistance = distanceEuclidean(location, obstacle);
      closest = obstacle;
    }
  }
  return closest;
}

template<typename CELL_T>
CELL_T &GridMap<CELL_T>::operator[](const util::Location &location)
{
  return ogm_[mapToIndex(location)];
}
template<typename T>
std::ostream &operator<<(std::ostream &out, const GridMap<T> &gridMap)
{
  //out << std::fixed << std::setprecision(1);
  for (uint index = 0; index < gridMap.ogm_.size(); index++) {
    if (!(index % gridMap.mapWidth_)) {
      out << "\n";
    }
    out <<gridMap.ogm_[index] << " | ";
  }
  return out;
}

template<typename CELL_T>
unsigned GridMap<CELL_T>::getCellWidth() const
{
  return mapWidth_;
}

template<typename CELL_T>
unsigned GridMap<CELL_T>::getCellHeight() const
{
  return mapHeight_;
}

template<typename CELL_T>
double GridMap<CELL_T>::getResolution() const
{
  return resolution_;
}

template<typename CELL_T>
void GridMap<CELL_T>::drawPath(std::ostream &out, std::vector<util::Location> path)
{
  out << std::fixed << std::setprecision(1);
  auto motionModel = util::Robot::getMotionModel();
  auto motionModelArrows = util::Robot::getMotionModelArrows();
  for (uint index = 0; index < ogm_.size(); index++) {
    if (!(index % mapWidth_)) {
      out << "\n";
    }
    auto iterator = std::find_if(path.begin(), path.end(), [&](const auto &location) { return this->indexToMap(index) == location; });
    if (iterator != path.end() && (iterator + 1) != path.end()) {
      auto movementLoc = *(iterator + 1) - indexToMap(index);
      auto iter2 = std::find_if(motionModel.begin(), motionModel.end(), [&](const auto &location) { return movementLoc == location; });
      auto distance = std::distance(motionModel.begin(), iter2);
      out << std::left << std::setw(4) << util::Robot::getMotionModelArrows()[distance] << " | ";
    } else {
      out << std::left << std::setw(4) << ogm_[index] << " | ";
    }
  }
}

template<typename CELL_T>
util::Location GridMap<CELL_T>::worldToMap(const Point &point) const noexcept(false)
{
  util::Location location;
  if (point.x < origin_.x || point.y < origin_.y) {
    throw util::OutOfBoundsException("Point out of origin");
  }
  location.x = static_cast<int>((point.x - origin_.x) / resolution_);
  location.y = static_cast<int>((point.y - origin_.y) / resolution_);
  throwIfOutOfBounds(location);
  return location;
}

template<typename CELL_T>
double GridMap<CELL_T>::getWorldWidth() const
{
  return getCellWidth() * resolution_;
}

template<typename CELL_T>
double GridMap<CELL_T>::getWorldHeight() const
{
  return getCellHeight() * resolution_;
}

template<typename CELL_T>
double GridMap<CELL_T>::worldDistanceManhattan(const util::Point &from, const util::Point &to) const
{
  double distanceX = std::abs(from.x - to.x);
  double distanceY = std::abs(from.y - to.y);
  return (distanceY + distanceX);
}

template<typename CELL_T>
double GridMap<CELL_T>::worldDistanceEuclidean(const util::Point &from, const util::Point &to) const
{
  double distanceX = std::pow(from.x - to.x, 2);
  double distanceY = std::pow(from.y - to.y, 2);
  return std::sqrt(distanceX + distanceY);
}

template<typename CELL_T>
double GridMap<CELL_T>::getOriginY() const
{
  return origin_.y;
}

template<typename CELL_T>
double GridMap<CELL_T>::getOriginX() const
{
  return origin_.x;
}

template<typename CELL_T>
const CELL_T &GridMap<CELL_T>::operator[](const Location &location) const
{
  return ogm_[mapToIndex(location)];
}
template<typename CELL_T>
util::Point GridMap<CELL_T>::mapToWorld(const util::Location &location) const
{
    util::Point p;
    p.x = origin_.x + (location.x + 0.5) * resolution_;
    p.y = origin_.y + (location.y + 0.5) * resolution_;
    return p;
}
template<typename CELL_T>
void GridMap<CELL_T>::plotMap()
{
#ifdef MATPLOTLIB
    namespace plt = matplotlibcpp;
    plt::imshow(reinterpret_cast<unsigned char*>(ogm_.data()),static_cast<int>(getCellWidth()), static_cast<int>(getCellHeight()),1);
    /*(3,4),(11,11),(22,11),(33,11)
        (11,30),(22,30), (33,30),(37,38)*/
    /*std::vector<double> x{2};
    std::vector<double> y{3};
    plt::plot(x,y,"go");
    x = std::vector<double>{11.5,22.5,33.5,11.5,22.5,33.5,36.5};
    y = std::vector<double>{11.5,11.5,11.5,30.5,30.5,30.5,36.5};
    plt::plot(x,y,"rx");*/
    plt::show();
#endif
}
template<typename CELL_T>
void GridMap<CELL_T>::plotPathOnMap(const std::vector<util::Point> &plan)
{
#ifdef MATPLOTLIB
    namespace plt = matplotlibcpp;
    std::vector<double> xplan;
    std::vector<double> yplan;
    for(const auto &item : plan) {
        xplan.push_back(item.x);
        yplan.push_back(item.y);
    }
    plt::plot(xplan,yplan);
    if(!xplan.empty()) {
        std::vector<double> startx{ *xplan.begin() };
        std::vector<double> starty{ *yplan.begin() };
        std::vector<double> goalx{ *xplan.rbegin() };
        std::vector<double> goaly{ *yplan.rbegin() };
        plt::plot(startx, starty, "ro");
        plt::plot(goalx, goaly, "rx");
    }
    plotMap();
#endif
}

    template<typename CELL_T>
    std::string GridMap<CELL_T>::getName() const {
        return name_;
    }

    template<typename CELL_T>
    void GridMap<CELL_T>::setName(const std::string &name) {
        name_ = name;
    }

}// namespace util


#endif//POTENTIALFIELDSPROJECT_GRIDMAP_H
