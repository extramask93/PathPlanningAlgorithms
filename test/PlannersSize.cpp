//
// Created by damian on 12.06.2020.
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

static std::pair<util::Point, util::Point>
getRandomStartAndGoalLocation(std::shared_ptr<util::GridMap<unsigned char>> map, double minDistance) {
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
        distance = map->worldDistanceEuclidean(startLocation, goalLocation);
    } while (!planner.isStartAndGoalValid(startLocation, goalLocation) || (distance < minDistance));
    return std::make_pair(startLocation, goalLocation);
}

static std::vector<std::pair<util::Point, util::Point>>
getRandomStartAndGoalLocations(std::shared_ptr<util::GridMap<unsigned char>> map, int nrOfPoints) {
    std::vector<std::pair<util::Point, util::Point>> result;
    auto minDistance = ceil(map->getWorldWidth() * 0.2);
    for (int i = 0; i < nrOfPoints; i++) {
        result.push_back(getRandomStartAndGoalLocation(map, minDistance));
    }
    return result;
}

static std::vector<std::string> Sizemaps = {
        "/home/damian/PathPlanningAlgorithms/resources/maps/wielkosci/map100x100.pgm",
        "/home/damian/PathPlanningAlgorithms/resources/maps/wielkosci/map500x500.pgm",
        "/home/damian/PathPlanningAlgorithms/resources/maps/wielkosci/map1000x1000.pgm",
        "/home/damian/PathPlanningAlgorithms/resources/maps/wielkosci/map2000x2000.pgm",
};

TEST_CASE("Dijkstra planner size", "[Dijkstra]") {
    for (const auto &mapPath : Sizemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner = astar::AStar(map);
        util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
        planner.setHeuristic(astar::AStar::HeuristicType::NO_HEURISTIC);
        planner.setName("Dijkstra");
        auto benchmarker = util::Benchmarker("size-dijkstra.csv");
        auto points = getRandomStartAndGoalLocations(map, 5);
        //auto path = planner.makePlan(points[0].first, points[0].second);
        //map->plotPathOnMap(path);
        benchmarker.evaluatePlanners(planner, map, points, 10);
    }
}

TEST_CASE("AStar planner size", "[AStar]") {
    for (const auto &mapPath : Sizemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner = astar::AStar(map);
        util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
        planner.setHeuristic(astar::AStar::HeuristicType::EUCLID);
        planner.setName("astar");
        auto benchmarker = util::Benchmarker("size-astar.csv");
        auto points = getRandomStartAndGoalLocations(map, 5);
        //auto path = planner.makePlan(points[0].first, points[0].second);
        //map->plotPathOnMap(path);
        benchmarker.evaluatePlanners(planner, map, points, 10);
    }
}

TEST_CASE("DepthFirst planner size", "[DepthFirst]") {
    for (const auto &mapPath : Sizemaps) {
        auto map = util::MapLoader::loadPGMMap(mapPath);
        auto planner = df::DepthFirst(map);
        util::Robot::movementType = util::Robot::MovementType::EUCLIDEAN;
        planner.setName("df");
        auto benchmarker = util::Benchmarker("size-df.csv");
        auto points = getRandomStartAndGoalLocations(map, 5);
        benchmarker.evaluatePlanners(planner, map, points, 10);
    }
}

static std::vector<std::pair<util::Point, util::Point>> points1 = {
        {{154.889, 298.791}, {436.196, 491.51}},
        {{233.702, 437.872}, {148.034, 65.6455}},
        {{421.409, 329.518},{297.72,  218.177}},
        {{178.125, 293.565},{74.7357, 85.6193}},
        {{274.408, 63.4859},{39.8963, 117.519}}
};

static std::vector<std::pair<util::Point, util::Point>> points2 = {
        {{659.965, 214.953},{203.047, 382.865}},
        {{224.873, 114.649},{583.017, 232.641}},
        {{556.518, 523.811},{350.958, 705.332}},
        {{820.704, 134.563},{604.726, 283.148}},
        {{664.655, 377.441},{747.582, 140.576}}
};

static std::vector<std::pair<util::Point, util::Point>> points3 = {
        {{776.099, 1732.98},{1913.71, 973.107}},
        {{211.866, 680.075},{1671.14, 930.883}},
        {{225.329, 1087.84},{1441.6,  1731.63}},
        {{5.83217, 590.884},{1973.78, 476.665}},
        {{1466.79, 1924.39},{1630.91, 1388.45}}
};

TEST_CASE("RRT planner size", "[RRT]") {
    //for(const auto &mapPath : Sizemaps) {
    auto map = util::MapLoader::loadPGMMap(Sizemaps[3]);
    auto planner = rrt::RrtPlanner(map);
    planner.setName("rrt");
    auto benchmarker = util::Benchmarker("size-rrt.csv");
    auto points = points3;//getRandomStartAndGoalLocations(map, 5);
    //auto plan = planner.makePlan(util::Point{659.965,214.953}, util::Point{203.047,382.865});
    //map->plotPathOnMap(plan);
    benchmarker.evaluatePlanners(planner, map, points, 100);
    //}
}

TEST_CASE("RRTStar planner size", "[RRTStar]") {
    //for(const auto &mapPath : Sizemaps) {
    auto map = util::MapLoader::loadPGMMap(Sizemaps[3]);
    auto planner = rrt::RrtStar(map);
    planner.setName("rrt-star");
    planner.setRunToMaxIterations(true);
    planner.setGamma(50);
    auto benchmarker = util::Benchmarker("size-rrt-star.csv");
    auto points = points3;//getRandomStartAndGoalLocations(map, 5);
    //auto path = planner.makePlan(points[0].first, points[0].second);
    //map->plotPathOnMap(path);
    benchmarker.evaluatePlanners(planner, map, points, 100);
    //}
}

TEST_CASE("RPM planner size", "[PRM]") {
    //for(const auto &mapPath : Sizemaps) {
    auto map = util::MapLoader::loadPGMMap(Sizemaps[3]);
    auto planner = prm::Prm(map, 900); //500 for 1000x1000, 900 for 2000x2000
    planner.setName("prm");
    auto benchmarker = util::Benchmarker("size-prm.csv");
    auto points = points3;//getRandomStartAndGoalLocations(map, 5);
    //auto path = planner.makePlan(points[0].first, points[0].second);
    //map->plotPathOnMap(path);
    benchmarker.evaluatePlanners(planner, map, points, 100);
    //}
}

TEST_CASE("Potential fields planner size", "[PF]") {
    //for(const auto &mapPath : Sizemaps) {
    auto map = util::MapLoader::loadPGMMap(Sizemaps[1]);
    auto planner = pf::PotentialFieldsPlanner(map);
    planner.setName("pf");
    auto benchmarker = util::Benchmarker("size-pf.csv");
    auto points = points1;//getRandomStartAndGoalLocations(map, 5);
    //auto path = planner.makePlan(points[0].first, points[0].second);
    //map->plotPathOnMap(path);
    benchmarker.evaluatePlanners(planner, map, points, 1);
    //}
}

TEST_CASE("Ant colony planner size", "[ANT]") {
    //for (const auto &mapPath : Sizemaps) {
        auto map = util::MapLoader::loadPGMMap(Sizemaps[0]);
        auto planner = AntColony(map);
        planner.setName("ant");
        auto benchmarker = util::Benchmarker("size-ant.csv");
        auto points = getRandomStartAndGoalLocations(map, 5);
        //auto path = planner.makePlan(points[0].first, points[0].second);
        //map->plotPathOnMap(path);
        benchmarker.evaluatePlanners(planner, map, points,1);
    //}
}
