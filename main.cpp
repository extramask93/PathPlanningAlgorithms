//
// Created by damian on 19.04.2020.
//
#include "PotentialFieldsPlanner.h"
int  main(int argc, char **argv) {
    std::vector<bool> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0};
    pf::PotentialFieldsPlanner planner(ogm, 7,7, 1.0);
    planner.makePlan(0, ogm.size()-1);
    planner.printMapOfPotentials();
    planner.printPath();
    return 0;
}