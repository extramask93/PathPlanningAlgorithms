//
// Created by damian on 26.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_ROADPOINT_H
#define POTENTIALFIELDSPROJECT_ROADPOINT_H
#include <Point.h>
namespace util {
struct RoadPoint : public util::Point {
    RoadPoint() : Point(0.0,0.0){}
    RoadPoint(double xx, double yy) : Point(xx,yy) {}
    std::vector<util::Point> edgesTo;
};
}
#endif //POTENTIALFIELDSPROJECT_ROADPOINT_H
