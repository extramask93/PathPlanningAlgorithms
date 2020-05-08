//
// Created by damian on 07.05.2020.
//
#include <RrtPlanner.h>
#include "catch.hpp"
#include "GridMap.h"
#include "rrtstar.h"
#include "PRM.h"
#include "MapLoader.h"
#include "Benchmarker.h"
double resolution = 1.0;
std::vector testPointsMap{
    util::Point{ 11 * resolution, 11 * resolution },
    util::Point{ 22 * resolution, 11 * resolution },
    util::Point{ 33 * resolution, 11 * resolution },
    util::Point{ 11 * resolution, 30 * resolution },
    util::Point{ 22 * resolution, 30 * resolution },
    util::Point{ 33 * resolution, 30 * resolution },
    util::Point{ 37 * resolution, 37 * resolution },
};
TEST_CASE("RRT planner, cluttered map", "[RRT]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    auto rrtPlanner = rrt::RrtPlanner(map);
    for (int pointIdx = 0; pointIdx < testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 10; ) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = rrtPlanner.makePlan({ 3 * resolution, 4 * resolution }, testPointsMap[pointIdx]);
            if(plan.empty()) {
                continue;
            }
            benchmarker.stop(plan);
            i++;
        }

        benchmarker.printAverages();
        benchmarker.save("rrt-point" + std::to_string(pointIdx) + ".dat");
    }
}

TEST_CASE("RRT* gamma value, cluttered map", "[RRTStar]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    for(int gamma = 10; gamma <= 100; gamma+=10) {
        auto rrtPlanner = rrt::RrtStar(map);
        rrtPlanner.setGamma(gamma);
        util::Benchmarker benchmarker;
        for (int i = 0; i < 10;) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = rrtPlanner.makePlan({ 3 * resolution, 4 * resolution }, testPointsMap[testPointsMap.size()-1]);
            if (plan.empty()) {
                std::cout<<"fail\n";
                continue;
            }
            benchmarker.stop(plan);
            i++;
        }
        benchmarker.printAverages();
        benchmarker.save(gamma,"rrtstar-samples.dat");
    }
}
TEST_CASE("RRT* planner, cluttered map", "[RRTStar]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    auto rrtPlanner = rrt::RrtStar(map);
    for (int pointIdx = 0; pointIdx < testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 10; ) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = rrtPlanner.makePlan({ 3 * resolution, 4 * resolution }, testPointsMap[pointIdx]);
            if(plan.empty()) {
                continue;
            }
            benchmarker.stop(plan);
            i++;
        }

        benchmarker.printAverages();
        benchmarker.save("rrt-point" + std::to_string(pointIdx) + ".dat");
    }
}

TEST_CASE("PRM planner, cluttered map", "[PRM]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    auto rrtPlanner = prm::Prm(map,50);
    for (int pointIdx = 0; pointIdx < testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 200; ) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = rrtPlanner.makePlan({ 3 * resolution, 4 * resolution }, testPointsMap[pointIdx]);
            if(plan.empty()) {
                std::cout<<"failed";
                continue;
            }
            benchmarker.stop(plan);
            i++;
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx,"prm.dat");
    }
}
TEST_CASE("PRM time,length vs nr of Samples, cluttered map", "[PRM]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    for(int nrOfSamples = 50; nrOfSamples < 2000; nrOfSamples+=50) {
        auto rrtPlanner = prm::Prm(map, nrOfSamples);
        util::Benchmarker benchmarker;
        for (int i = 0; i < 10;) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = rrtPlanner.makePlan({ 3 * resolution, 4 * resolution }, testPointsMap[testPointsMap.size()-1]);
            if (plan.empty()) {
                std::cout<<"fail\n";
                continue;
            }
            benchmarker.stop(plan);
            i++;
        }
        benchmarker.printAverages();
        benchmarker.save(nrOfSamples,"prm-samples.dat");
    }
}
