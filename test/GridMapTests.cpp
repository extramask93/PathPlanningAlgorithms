//
// Created by damian on 25.04.2020.
//
#include <catch2/catch.hpp>
#include <GridMap.h>
TEST_CASE( "Creating grid map via constructor and vector", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0};
    util::GridMap<uint8_t> map(ogm,7,7);
}
TEST_CASE( "Throwing when accessing out of bounds element", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0};
    util::GridMap<uint8_t> map(ogm,7,7);
    REQUIRE_THROWS_AS(map.isFree({-1,5}), util::OutOfBoundsException);
}
TEST_CASE( "Correctly assessing if cell is free", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0};
    util::GridMap<uint8_t> map(ogm,7,7);
    REQUIRE(map.isFree({0,0}));
    REQUIRE(map.isFree({6,6}));
}
TEST_CASE( "Correctly assessing if cell is not free", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,1};
    util::GridMap<uint8_t> map(ogm,7,7);
    REQUIRE_FALSE(map.isFree({2,3}));
    REQUIRE_FALSE(map.isFree({6,6}));
}
TEST_CASE( "Correctly calculating euklidean distance", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,1};
    util::GridMap<uint8_t> map(ogm,7,7,2);
    REQUIRE(map.distanceEuclidean({0, 0}, {6, 6}) == Approx(2 * 8.485281));
}
TEST_CASE( "Correctly calculating manhattan distance", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,1};
    util::GridMap<uint8_t> map(ogm,7,7,2);
    REQUIRE(map.distanceManhattan({0,0}, {6,6}) ==  Approx(2*12.0));
}
TEST_CASE( "Correctly calculating location from index", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,1};
    util::GridMap<uint8_t> map(ogm,7,7,2);
    map.indexToMap(9);
    REQUIRE(map.indexToMap(9) == util::Location{2,1});
}
TEST_CASE( "Finding all obstacles", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 1, 1, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,1};
    util::GridMap<uint8_t> map(ogm,7,7,2);
    REQUIRE(map.findAllObstacles().size() == 5);
}
TEST_CASE( "Finding closest obstacle on obstacle free map", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0};
    util::GridMap<uint8_t> map(ogm,7,7,2);
    REQUIRE(map.findClosestObstacle({0,0}) == boost::none);
}
TEST_CASE( "Using operator [] return correct value", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,0,
                           0, 0, 0, 0, 0,0,1};
    util::GridMap<uint8_t> map(ogm,7,7,2);
    REQUIRE(map[{0,0}] == 0);
    REQUIRE(map[{6,6}] == 1);
}
TEST_CASE( "Getting origin x and y", "[Util,GridMap]" ) {

    std::vector<uint8_t> ogm {0, 0, 0, 0, 0,0,0,
                              0, 0, 0, 0, 0,0,0,
                              0, 0, 0, 0, 0,0,0,
                              0, 0, 0, 0, 0,0,0,
                              0, 0, 0, 0, 0,0,0,
                              0, 0, 0, 0, 0,0,0,
                              0, 0, 0, 0, 0,0,1};
    util::GridMap<uint8_t> map(ogm,7,7,2);
    REQUIRE(map.getOriginX() == 0);
    REQUIRE(map.getOriginY() == 0);
}
