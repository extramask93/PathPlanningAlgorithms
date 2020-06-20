//
// Created by damian on 11.06.2020.
//

#include <catch2/catch.hpp>
#include <GridMap.h>
#include <AStar.h>
#include <MapLoader.h>
#include <Benchmarker.h>
#include <matplotlibcpp.h>
#include <memory>
#include <utility>
#include <DepthFirst.h>
#include <RrtPlanner.h>
#include <rrtstar.h>
#include <PotentialFieldsPlanner.h>
#include <AntColony.h>
#include <PRM.h>

static std::pair<util::Point, util::Point> getRandomStartAndGoalLocation(std::shared_ptr<util::GridMap<unsigned char>> map, double minDistance)
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
    double distance = 0.0;
    do {
        startLocation = util::Point{xDistribution(randomGenerator), yDistribution(randomGenerator)};
        goalLocation = util::Point{xDistribution(randomGenerator), yDistribution(randomGenerator)};
        distance = map->worldDistanceEuclidean(startLocation,goalLocation);
    } while (!planner.isStartAndGoalValid(startLocation, goalLocation) || (distance < minDistance));
    return std::make_pair(startLocation, goalLocation);
}
static std::vector<std::pair<util::Point, util::Point>>
getRandomStartAndGoalLocations(std::shared_ptr<util::GridMap<unsigned char>> map, int nrOfPoints) {
    std::vector<std::pair<util::Point, util::Point>> result;
    auto minDistance = ceil(map->getWorldWidth()*0.2);
    for(int i =0; i < nrOfPoints; i++) {
        result.push_back(getRandomStartAndGoalLocation(map,minDistance));
    }
    return result;
}

static std::vector<std::string> Mazemaps = {
        "/home/damian/PathPlanningAlgorithms/resources/maps/labirynty/labirynt3.pgm",
        "/home/damian/PathPlanningAlgorithms/resources/maps/labirynty/labirynt4.pgm",
        "/home/damian/PathPlanningAlgorithms/resources/maps/labirynty/labirynt5.pgm",
};

TEST_CASE("Dijkstra planner maze", "[Dijkstra]")
{
    for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner = astar::AStar(map);
        util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
        planner.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
        planner.setName("Dijkstra");
        auto benchmarker = util::Benchmarker("maze-dijkstra.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        //auto path = planner.makePlan(points[0].first, points[0].second);
        //map->plotPathOnMap(path);
        benchmarker.evaluatePlanners(planner, map, points,10);
    }
}

TEST_CASE("AStar planner maze", "[AStar]")
{
    for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner = astar::AStar(map);
        util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
        planner.setHeuristic(astar::AStar::HeuristicType::EUCLID);
        planner.setName("astar");
        auto benchmarker = util::Benchmarker("maze-astar.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        auto path = planner.makePlan(points[0].first, points[0].second);
        map->plotPathOnMap(path);
        //benchmarker.evaluatePlanners(planner, map, points,10);
    }
}
TEST_CASE("DepthFirst planner maze", "[DepthFirst]")
{
    for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner  = df::DepthFirst(map);
        util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
        planner.setName("df");
        auto benchmarker = util::Benchmarker("maze-df.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        auto path = planner.makePlan(points[0].first, points[0].second);
        map->plotPathOnMap(path);
        //benchmarker.evaluatePlanners(planner, map, points,10);
    }
}

TEST_CASE("RRT planner maze", "[RRT]")
{
    //for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(Mazemaps[0]);
        auto planner  = rrt::RrtPlanner(map);
        planner.setName("rrt");
        auto benchmarker = util::Benchmarker("maze-rrt.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        auto path = planner.makePlan(points[0].first, points[0].second);
        map->plotPathOnMap(path);
        //benchmarker.evaluatePlanners(planner, map, points,1);
    //}
}

TEST_CASE("RRTStar planner maze", "[RRTStar]")
{
    for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner  = rrt::RrtStar(map);
        planner.setName("rrt-star");
        planner.setRunToMaxIterations(true);
        planner.setGamma(50);
        auto benchmarker = util::Benchmarker("maze-rrt-star.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        //auto path = planner.makePlan(points[0].first, points[0].second);
        //map->plotPathOnMap(path);
        benchmarker.evaluatePlanners(planner, map, points,100);
    }
}
TEST_CASE("RPM planner maze", "[PRM]")
{
    for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner  = prm::Prm(map,1000);
        planner.setName("prm");
        auto benchmarker = util::Benchmarker("maze-prm-star.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        auto path = planner.makePlan(points[0].first, points[0].second);
        map->plotPathOnMap(path);
        //benchmarker.evaluatePlanners(planner, map, points,100);
    }
}
TEST_CASE("Potential fields planner maze", "[PF]")
{
    for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner  = pf::PotentialFieldsPlanner(map);
        planner.setName("pf");
        auto benchmarker = util::Benchmarker("maze-pf.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        auto path = planner.makePlan(points[0].first, points[0].second);
        map->plotPathOnMap(path);
        //benchmarker.evaluatePlanners(planner, map, points,10);
    }
}
TEST_CASE("Ant colony planner maze", "[ANT]")
{
    for(const auto &mapPath : Mazemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner  = AntColony(map);
        planner.setName("ant");
        auto benchmarker = util::Benchmarker("maze-ant.csv");
        auto points = getRandomStartAndGoalLocations(map,5);
        auto path = planner.makePlan(points[0].first, points[0].second);
        map->plotPathOnMap(path);
        //benchmarker.evaluatePlanners(planner, map, points,1);
    }
}
