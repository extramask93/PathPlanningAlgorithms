//
// Created by damian on 11.05.2020.
//
#include "catch.hpp"
#include "GridMap.h"
#include "MapLoader.h"
#include "Benchmarker.h"
#include "AntColony.h"
#include "PotentialFieldsPlanner.h"
extern double resolution;
extern std::vector<util::Point> testPointsMap;
TEST_CASE("ACO planner, cluttered map", "[ACO]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    auto rrtPlanner = AntColony(map);
    for (int pointIdx = 1; pointIdx < testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1; ) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = rrtPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            if(plan.empty()) {
                std::cout<<"failed\n";
                continue;
            }
            benchmarker.stop(plan);
            i++;
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx,"aco.dat");
    }

}

TEST_CASE("Potential fields planner, cluttered map", "[ACO]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    auto rrtPlanner = pf::PotentialFieldsPlanner(map);
    for (int pointIdx = 0; pointIdx < testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 100; ) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = rrtPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            benchmarker.stop(plan);
            i++;
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx,"potfield.dat");
    }

}
