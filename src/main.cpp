//
// Created by damian on 19.04.2020.
//
#include <PotentialFieldsPlanner.h>
#include <PRM.h>
#include <GridMap.h>
#include <Robot.h>
#include <iostream>
#include <DepthFirst.h>
#include <AntColony.h>
#include "RrtPlanner.h"
#include "rrtstar.h"
#include "AStar.h"
#include "Benchmarker.h"
#include "MapLoader.h"
int  main(int , char **)
{

    double resolution = 1.0;
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), resolution);

    int n_ants = 10, iterations = 50;
    float alpha = 1, beta =0.7, evap_rate = 0.3, Q = 10;
    AntColony new_ant_colony(n_ants, alpha, beta, evap_rate, iterations, Q);
    std::vector testPointsMap{
        util::Point{ 11 * resolution, 11 * resolution },
        util::Point{ 22 * resolution, 11 * resolution },
        util::Point{ 33 * resolution, 11 * resolution },
        util::Point{ 11 * resolution, 30 * resolution },
        util::Point{ 22 * resolution, 30 * resolution },
        util::Point{ 33 * resolution, 30 * resolution },
        util::Point{ 37 * resolution, 37 * resolution },
    };
    auto plan = new_ant_colony.makePlan(map,{ 2 * resolution, 3 * resolution }, testPointsMap[3]);
    map.plotPathOnMap(plan);
    return 0;
    std::vector<int> gammas {5,50,100};
    for(auto point : testPointsMap) {
    for (int gamma : gammas) {
            auto rrtPlanner = rrt::RrtStar(map);
            //rrtPlanner.setGamma(gamma);
            util::Benchmarker benchmarker;
            for (int i = 0; i < 100;) {
                std::vector<util::Point> plan;
                benchmarker.start();
                rrtPlanner.setRunToMaxIterations(true);
                rrtPlanner.setGamma(gamma);
                plan = rrtPlanner.makePlan({ 3 * resolution, 4 * resolution }, point);
                if (plan.empty()) {
                    std::cout << "fail\n";
                    continue;
                }
                benchmarker.stop(plan);
                i++;
                //map.plotPathOnMap(plan);
            }
            benchmarker.printAverages();
            benchmarker.save(gamma, "rrtstar2-samples.dat");
        }
    }
        /*std::vector<unsigned char> ogm{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    util::GridMap<unsigned char> map(ogm, 7, 7);//(std::get<0>(temp), std::get<1>(temp),std::get<2>(temp),1.0);
    auto rrtplanner = df::DepthFirst(map);
    std::vector<util::Point> plan;
    {
        plan = rrtplanner.makePlan({ 0, 0 }, util::Point{ map.getCellWidth() - 1 * 1.0, map.getCellHeight() - 1 * 1.0 });
    }
     (3,4),(11,11),(22,11),(33,11)
      (11,30),(22,30), (33,30),(37,38)*/
        //map.plotMap();
        //map.plotPathOnMap(plan);
    return 0;
}