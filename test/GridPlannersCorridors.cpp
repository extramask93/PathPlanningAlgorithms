//
// Created by damian on 07.06.2020.
//
#include <catch2/catch.hpp>
#include <GridMap.h>
#include <AStar.h>
#include <MapLoader.h>
#include <Benchmarker.h>
#include <memory>
#include <utility>
std::pair<util::Point, util::Point> getRandomStartAndGoalLocation(std::shared_ptr<util::GridMap<unsigned char>> map)
{
    auto planner = astar::AStar(map);
    util::Point startLocation;
    util::Point goalLocation;
    static std::mt19937 randomGenerator(10);
    using Distribution = std::uniform_real_distribution<double>;
    Distribution xDistribution(map->getOriginX(),
                                      map->getOriginX() + map->getWorldWidth());
    Distribution yDistribution(map->getOriginY(),
                                      map->getOriginY() + map->getWorldHeight());
    do {
        startLocation = util::Point{xDistribution(randomGenerator), yDistribution(randomGenerator)};
        goalLocation = util::Point{xDistribution(randomGenerator), yDistribution(randomGenerator)};
    } while (!planner.isStartAndGoalValid(startLocation, goalLocation));
    return std::make_pair(startLocation, goalLocation);
}
std::vector<std::pair<util::Point, util::Point>>
    getRandomStartAndGoalLocations(std::shared_ptr<util::GridMap<unsigned char>> map, int nrOfPoints) {
    std::vector<std::pair<util::Point, util::Point>> result;
    for(int i =0; i < nrOfPoints; i++) {
        result.push_back(getRandomStartAndGoalLocation(map));
    }
    return result;
}

TEST_CASE("Dijkstra planner corridors", "[Dijkstra]")
{
    auto map = util::MapLoader::loadPGMMap("/home/damian/PathPlanningAlgorithms/resources/maps/wielkosci/map500x500.pgm");
    auto aStarPlanner = astar::AStar(map);
    util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
    aStarPlanner.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
    auto benchmarker = util::Benchmarker();
    auto points = getRandomStartAndGoalLocations(map,5);
    benchmarker.evaluatePlanners(aStarPlanner, map, points,10);
}
