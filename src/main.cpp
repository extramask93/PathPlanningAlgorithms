//
// Created by damian on 19.04.2020.
//
#include <PotentialFieldsPlanner.h>
#include <PRM.h>
#include <GridMap.h>
#include <Robot.h>
#include <iostream>
#include "RrtPlanner.h"
#include "AntColony.h"
#include "AStar.h"
#include "Benchmarker.h"
#include "MapLoader.h"
int  main(int , char **)
{

    std::vector<unsigned char> ogm{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    auto temp = util::MapLoader::loadMap("map2.csv");
    util::GridMap<unsigned char> map(ogm, 7, 7);//(std::get<0>(temp), std::get<1>(temp),std::get<2>(temp),1.0);
    auto rrtplanner = astar::AStar(map);
    std::vector<util::Point> plan;
    {
        util::Benchmarker bench(plan, util::Point(0, 0), util::Point(6, 6));
        plan = rrtplanner.makePlan({ 0, 0 }, util::Point{ map.getCellWidth() - 1 * 1.0, map.getCellHeight() - 1 * 1.0 });
    }
    map.plotPathOnMap(plan);
return 0;
}