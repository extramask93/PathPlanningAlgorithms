//
// Created by damian on 19.04.2020.
//
#include <PotentialFieldsPlanner.h>
#include <PRM.h>
#include <GridMap.h>
#include <Robot.h>
#include <iostream>
#include "rrtstar.h"
#include "matplotlibcpp.h"
int  main(int , char **) {

    namespace plt = matplotlibcpp;
    std::vector<int> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0};
    util::GridMap<int> map(ogm, 7,7,1.0);
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
    return 0;
}