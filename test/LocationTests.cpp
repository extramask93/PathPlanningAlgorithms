//
// Created by damian on 25.04.2020.
//
#include <catch2/catch.hpp>
#include <Location.h>
TEST_CASE( "Default location is 0,0", "[Util,Location]" ) {
    util::Location emptyLocation;
    REQUIRE(((emptyLocation.x == 0) && (emptyLocation.y ==0)));
}
TEST_CASE( "Setting location via initialization list", "[Util,Location]" ) {
    util::Location emptyLocation {1,2};
    REQUIRE(((emptyLocation.x == 1) && (emptyLocation.y ==2)));
}
TEST_CASE( "Setting via member access works", "[Util,Location]" ) {
    util::Location emptyLocation {1,2};
    emptyLocation.x = 4;
    REQUIRE_FALSE((emptyLocation.x == 1));
}
