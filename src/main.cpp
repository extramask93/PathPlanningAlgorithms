//
// Created by damian on 19.04.2020.
//
#include <PotentialFieldsPlanner.h>
#include <PRM.h>
#include <GridMap.h>
#include <Robot.h>
#include <iostream>

int  main(int , char **) {
    std::vector<int> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0};
    util::GridMap<int> map(ogm, 7,7,1.0);
    pf::PotentialFieldsPlanner planner(map);
    auto path = planner.makePlan(util::Location{0,0}, util::Location{6,6}, util::Robot{});
    prm::Prm planner2(map,24);
    auto plan = planner2.makePlan({0,0},{6,6});
    std::cout <<map<<'\n';
    map.drawPath(std::cout, path);
    return 0;
}