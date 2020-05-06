//
// Created by damian on 05.05.2020.
//
#include "catch.hpp"
#include "GridMap.h"
#include "MapLoader.h"
#include "AStar.h"
#include "Benchmarker.h"
#include "DepthFirst.h"
std::vector testPointsMap2 {
    util::Point{30,30},
    util::Point{11,25},
    util::Point{30,10},
    util::Point{15,18}
};
TEST_CASE("Dijkstra planner with manhattan, cluttered map", "[Dijkstra,Manhattan]")
{
    auto temp = util::MapLoader::loadMap("map2.csv");
    util::Robot::movementType = util::Robot::MovementType::MANHATTAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
    for(int pointIdx = 0; pointIdx<testPointsMap2.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 0, 0 }, testPointsMap2[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save("Dijkstra-manhattan-map2-point"+std::to_string(pointIdx)+".dat");
    }
}
TEST_CASE("Dijkstra planner with euclidean, cluttered map", "[Dijkstra,Euclidean]")
{
    auto temp = util::MapLoader::loadMap("map2.csv");
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
    for(int pointIdx = 0; pointIdx<testPointsMap2.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 0, 0 }, testPointsMap2[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save("Dijkstra-euclidean-map2-point"+std::to_string(pointIdx)+".dat");
    }
}

TEST_CASE("AStar planner with manhattan, cluttered map", "[AStar,Manhattan]")
{
    auto temp = util::MapLoader::loadMap("map2.csv");
    util::Robot::movementType = util::Robot::MovementType::MANHATTAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::MANHATTAN);
    for(int pointIdx = 0; pointIdx<testPointsMap2.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 0, 0 }, testPointsMap2[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save("AStar-manhattan-map2-point"+std::to_string(pointIdx)+".dat");
    }
}

TEST_CASE("AStar planner with euclidean, cluttered map", "[AStar,Euclidean]")
{
    auto temp = util::MapLoader::loadMap("map2.csv");
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = astar::AStar(map);
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::EUCLID);
    for(int pointIdx = 0; pointIdx<testPointsMap2.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 0, 0 }, testPointsMap2[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save("AStar-euclidean-map2-point"+std::to_string(pointIdx)+".dat");
    }
}

TEST_CASE("DepthFirst planner with manhattan, cluttered map", "[DFS,Manhattan]")
{
    auto temp = util::MapLoader::loadMap("map2.csv");
    util::Robot::movementType = util::Robot::MovementType::MANHATTAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = df::DepthFirst(map);
    for(int pointIdx = 0; pointIdx<testPointsMap2.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 0, 0 }, testPointsMap2[pointIdx]);
            benchmarker.stop(plan);
        }
        benchmarker.printAverages();
        benchmarker.save("DFS-manhattan-map2-point"+std::to_string(pointIdx)+".dat");
    }
}

TEST_CASE("DepthFirst planner with euclidean, cluttered map", "[DFS,Euclidean]")
{
    auto temp = util::MapLoader::loadMap("map2.csv");
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    util::GridMap<unsigned char> map(std::get<0>(temp), std::get<1>(temp), std::get<2>(temp), 1.0);
    auto aStarPlanner = df::DepthFirst(map);
    for(int pointIdx = 0; pointIdx<testPointsMap2.size(); pointIdx++) {
        util::Benchmarker benchmarker;
        for (int i = 0; i < 1000; i++) {
            std::vector<util::Point> plan;
            benchmarker.start();
            plan = aStarPlanner.makePlan({ 0, 0 }, testPointsMap2[pointIdx]);
            benchmarker.stop(plan);
        }

        benchmarker.printAverages();
        benchmarker.save("DFS-euclidean-map2-point"+std::to_string(pointIdx)+".dat");
    }
}
