//
// Created by damian on 01.05.2020.
//
#include "rrtstar.h"
#include "catch.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
TEST_CASE( "One should be able to create rrt star planner with ogm supplied", "[Planner, RRTStar]" ) {
std::vector<unsigned char> ogm {0, 0, 0, 0, 0,0,0,
                      0, 0, 0, 0, 0,0,0,
                      0, 0, 0, 0, 0,0,0,
                      0, 0, 1, 1, 0,0,0,
                      0, 0, 1, 1, 0,0,0,
                      0, 0, 0, 0, 0,0,0,
                      0, 0, 0, 0, 0,0,0};
util::GridMap<unsigned char> map(ogm, 7,7,1.0);
auto rrtplanner = rrt::RrtStar(map);
auto plan = rrtplanner.makePlan({0,0},{6,6});
auto obstacles = map.findAllObstacles();
std::vector<double> xplan;
std::vector<double> yplan;
std::vector<double> xobstacle;
std::vector<double> yobstacle;
for(const auto &item : obstacles) {
xobstacle.push_back(item.x);
yobstacle.push_back(item.y);
}
for(const auto &item : plan) {
xplan.push_back(item.x);
yplan.push_back(item.y);
}
plt::plot(xobstacle,yobstacle, "sk");
plt::plot(xplan,yplan);
plt::show();
}