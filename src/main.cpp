//
// Created by damian on 19.04.2020.
//
#include <PotentialFieldsPlanner.h>
#include <PRM.h>
#include <DepthFirst.h>
#include "RrtPlanner.h"
#include "rrtstar.h"
#include "AStar.h"
#include "Benchmarker.h"
#include "MapLoader.h"
//#include <matplotlibcpp.h>
int main(int, char **)
{

    double resolution = 1.0;
    auto map = util::MapLoader::loadPGMMap("pgmmap10x10.pgm");
    map.plotMap();
    std::vector<util::Point> testPointsMap{
        util::Point{ 11.5 * resolution, 11.5 * resolution },
        util::Point{ 22.5 * resolution, 11.5 * resolution },
        util::Point{ 33.5 * resolution, 11.5 * resolution },
        util::Point{ 11.5 * resolution, 30.5 * resolution },
        util::Point{ 22.5 * resolution, 30.5 * resolution },
        util::Point{ 33.5 * resolution, 30.5 * resolution },
        util::Point{ 36.5 * resolution, 36.5 * resolution },
    };
    auto new_ant_colony = astar::AStar(map);
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    new_ant_colony.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
    util::Benchmarker benchmarker;
    benchmarker.start();
    auto plan = new_ant_colony.makePlan({ 2.5, 3.5 }, testPointsMap[6]);
    benchmarker.stop(plan);
    benchmarker.printAverages();
    map.plotPathOnMap(plan);
    return 0;
}