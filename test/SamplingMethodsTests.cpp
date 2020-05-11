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
extern double resolution;
extern std::vector<util::Point> testPointsMap;
TEST_CASE("RRT planner, cluttered map", "[RRT]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    auto rrtPlanner = rrt::RrtStar(map);
    rrtPlanner.setGamma(2);
    rrtPlanner.setRunToMaxIterations(false);
    for (int pointIdx = 0; pointIdx < testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; ) {
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
        benchmarker.save(pointIdx,"rrt.dat");
    }

}

TEST_CASE("RRT* gamma value, cluttered map", "[RRTStar]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    std::vector<int> gammas {5,10,20,50,100};
    for(auto point : testPointsMap) {
        for (int gamma : gammas) {
            auto rrtPlanner = rrt::RrtStar(map);
            rrtPlanner.setGamma(gamma);
            util::Benchmarker benchmarker;
            for (int i = 0; i < 100;) {
                std::vector<util::Point> plan;
                benchmarker.start();
                rrtPlanner.setRunToMaxIterations(true);
                rrtPlanner.setGamma(gamma);
                plan = rrtPlanner.makePlan({ 2.5, 3.5 }, point);
                if (plan.empty()) {
                    std::cout << "fail\n";
                    continue;
                }
                benchmarker.stop(plan);
                i++;
            }
            benchmarker.printAverages();
            benchmarker.save(gamma, "rrtstar-samples.dat");
        }
    }
}
TEST_CASE("RRT* planner, cluttered map", "[RRTStar]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    auto rrtPlanner = rrt::RrtStar(map);
    rrtPlanner.setGamma(50);
    rrtPlanner.setRunToMaxIterations(true);
    for (int pointIdx = 0; pointIdx < testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; ) {
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
        benchmarker.save(pointIdx,"rrt-star.dat");
    }
}

TEST_CASE("PRM planner, cluttered map", "[PRM]")
{

    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = prm::Prm(map,50);
    for(int pointIdx = 0; pointIdx<testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            if(plan.empty()) {
                std::cout<<"failed\n";
                continue;
            }
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx,"prm.dat");
    }

}
TEST_CASE("PRM time,length vs nr of Samples, cluttered map", "[PRM]")
{
    /*
     * fail ratio for 10 : 0.383143
fail ratio for 20 : 0.0685714
fail ratio for 30 : 0.0104286
fail ratio for 40 : 0.001
fail ratio for 50 : 0
     */
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);
    for(int nrOfSamples = 10; nrOfSamples <= 50; nrOfSamples+=10) {
        int nrOfFails = 0;
        int totalTries = 0;
        for(int i =0; i<1000;i++) {
            auto rrtPlanner = prm::Prm(map, nrOfSamples);
            util::Benchmarker benchmarker;
            for (auto point : testPointsMap) {
                std::vector<util::Point> plan;
                plan = rrtPlanner.makePlan({ 2.5, 3.5 }, point);
                if (plan.empty()) {
                    nrOfFails++;
                    totalTries++;
                    continue;
                }
                totalTries++;
            }
        }
        std::cout<<"fail ratio for "<<nrOfSamples<<" : "<<1.0 * nrOfFails/totalTries<<std::endl;
    }
}
