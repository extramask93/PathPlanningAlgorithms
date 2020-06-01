//
// Created by damian on 01.06.2020.
//
#include <catch2/catch.hpp>
#include <MapLoader.h>
#include <filesystem>
#include <iostream>
SCENARIO("Loading map 10x10 from pgm file") {
    GIVEN("Map loader and map file") {
        auto loader = util::MapLoader();
        auto filePath = std::filesystem::current_path() / "pgmmap10x10.pgm";
        WHEN("Map is correctly loaded")
        {
            auto map = loader.loadPGMMap(filePath);
            THEN("Map has width equal to 10")
            {
                REQUIRE(map.getCellWidth() == 10);
            }
            AND_THEN("Map has height equal to 10")
            {
                REQUIRE(map.getCellHeight() == 10);
            }
            AND_THEN("fivth element is occupied")
            {
                REQUIRE(map.isFree(util::Location{ 5, 0 }) == false);
            }
        }
    }
}
