//
// Created by damian on 05.05.2020.
//
#include <unordered_map>
#include "catch.hpp"
#include "GridMap.h"
#include "MapLoader.h"
#include "AStar.h"
#include "Benchmarker.h"
#include "DepthFirst.h"
#include "Point.h"
double resolution = 1.0;
std::vector<util::Point> testPointsMap{
    util::Point{ 11.5 * resolution, 11.5 * resolution },
    util::Point{ 22.5 * resolution, 11.5 * resolution },
    util::Point{ 33.5 * resolution, 11.5 * resolution },
    util::Point{ 11.5 * resolution, 30.5 * resolution },
    util::Point{ 22.5 * resolution, 30.5 * resolution },
    util::Point{ 33.5 * resolution, 30.5 * resolution },
    util::Point{ 36.5 * resolution, 36.5 * resolution },
};
TEST_CASE("Dijkstra planner with manhattan, cluttered map", "[Dijkstra,Manhattan]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::Robot::movementType = util::Robot::MovementType::MANHATTAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
    for(int pointIdx = 0; pointIdx<testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx,"Dijkstra-manhattan.dat");
    }
}
TEST_CASE("Dijkstra planner with euclidean, cluttered map", "[Dijkstra,Euclidean]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
    for(int pointIdx = 0; pointIdx<testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx+1,"Dijkstra-euclidean.dat");
    }
}

TEST_CASE("AStar planner with manhattan, cluttered map", "[AStar,Manhattan]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::Robot::movementType = util::Robot::MovementType::MANHATTAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::MANHATTAN);
    for(int pointIdx = 0; pointIdx<testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx,"AStar-manhattan.dat");
    }
}

TEST_CASE("AStar planner with euclidean, cluttered map", "[AStar,Euclidean]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::EUCLID);
    for(int pointIdx = 0; pointIdx<testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx+1,"AStar-euclidean.dat");
    }
}

TEST_CASE("DepthFirst planner with manhattan, cluttered map", "[DFS,Manhattan]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::Robot::movementType = util::Robot::MovementType::MANHATTAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = df::DepthFirst(map);
    for(int pointIdx = 0; pointIdx<testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            benchmarker.stop(plan);
        }
        benchmarker.printAverages();
        benchmarker.save(pointIdx,"DFS-manhattan.dat");
    }
}

TEST_CASE("DepthFirst planner with euclidean, cluttered map", "[DFS,Euclidean]")
{
    auto temp = util::MapLoader::loadMap("map-warehouse.csv");
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = df::DepthFirst(map);
    for(int pointIdx = 0; pointIdx<testPointsMap.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 2.5, 3.5 }, testPointsMap[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save(pointIdx+1,"DFS-euclidean.dat");
    }
}
TEST_CASE("Grid planners, 4 connected vs grid size", "[All,GridSize]")
{
    std::vector<std::string> maps {
        "whitemap20x20.csv",
        "whitemap40x40.csv",
        "whitemap80x80.csv",
        "whitemap160x160.csv",
    };

    auto temp = util::MapLoader::loadMap(maps[0]);
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    std::unordered_map<std::string,std::shared_ptr<IPlanner>> planners{};
    planners.insert(std::make_pair("dijkstra",std::shared_ptr<IPlanner>(new astar::AStar(map))));
    planners.insert(std::make_pair("astar",std::shared_ptr<IPlanner>(new astar::AStar(map))));
    planners.insert(std::make_pair("df",std::shared_ptr<IPlanner>(new df::DepthFirst(map))));
    util::Robot::movementType = util::Robot::MovementType::MANHATTAN;
    for(auto mapName : maps) {
        temp = util::MapLoader::loadMap(mapName);
        map = util::GridMap<unsigned char >(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
        util::Options options;
        for(auto planner : planners) {
            options.heuristic = planner.first == "dijkstra" ?(int)astar::AStar::HeuristicType::NO_HEURISTIC: (int)astar::AStar::HeuristicType::MANHATTAN;
            planner.second->initialize(map,options);
            util::Benchmarker benchmarker;
            benchmarker.start();
            auto plan = planner.second->makePlan( {0, 0 }, {map.getCellWidth()-1.0, map.getCellHeight()-1.0});
            benchmarker.stop(plan);
            benchmarker.printAverages();
            benchmarker.saveVar("differentMapGrid.dat","ds",map.getCellHeight(),planner.first.c_str());
        }

    }
}
