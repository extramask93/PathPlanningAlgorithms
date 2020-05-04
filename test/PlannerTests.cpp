//
// Created by damian on 19.04.2020.
//
#include "catch.hpp"
#include "PotentialFieldsPlanner.h"
#include "matplotlibcpp.h"
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <algorithm>
#include <tuple>
#include <PRM.h>
#include <MapLoader.h>

namespace plt = matplotlibcpp;

TEST_CASE( "One should be able to create prm planner with ogm supplied", "[Planner, PRM]" ) {
    std::vector<unsigned char> ogm {0, 0, 0, 0, 0,0,0,
                          0, 0, 0, 0, 0,0,0,
                          0, 0, 0, 0, 0,0,0,
                          0, 0, 1, 1, 0,0,0,
                          0, 0, 1, 1, 0,0,0,
                          0, 0, 0, 0, 0,0,0,
                          0, 0, 0, 0, 0,0,0};
    util::GridMap<unsigned char> map(ogm, 7,7,1.0);
    prm::Prm prmPlanner(map,24);
    auto plan = prmPlanner.makePlan({0,0},{6,6});
    auto samples = prmPlanner.samples_;
    auto obstacles = map.findAllObstacles();
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> xplan;
    std::vector<double> yplan;
    std::vector<double> xobstacle;
    std::vector<double> yobstacle;
    for(const auto &item : obstacles) {
        xobstacle.push_back(item.x);
        yobstacle.push_back(item.y);
    }
    for(const auto &item : plan) {
        xplan.push_back(item.x);
        yplan.push_back(item.y);
    }
    for(const auto &sample : samples) {
        x.push_back(sample.x);
        y.push_back(sample.y);
    }
    plt::plot(x,y, "go");
    plt::plot(xobstacle,yobstacle, "sk");
    plt::plot(xplan,yplan);
    plt::show();
}
TEST_CASE( "One should be able to create planner with ogm supplied", "[Planner]" ) {
    std::vector<int> ogm;
    unsigned int width =3;
    unsigned int height =2;
    auto tu = util::MapLoader::loadMap("/home/damian/Planning/map.csv");
    ogm = std::get<0>(tu);
    width = std::get<1>(tu);
    height = std::get<2>(tu);
    plt::figure_size(width,height);
    std::cout<<width<<"x"<<height;
    std::vector<int> xv0;
    std::vector<int> yv0;
    std::vector<int> xv1;
    std::vector<int> yv1;
    for(int x = 0 ;x <width; x++) {
        for(int y = 0; y<height; y++) {
            if(ogm[y*width +x] == 1) {
                xv1.push_back(x);
                yv1.push_back(y);
            } else {
                xv0.push_back(x);
                yv0.push_back(y);
            }
        }
    }
    plt::plot(xv0,yv0, "go");
    plt::plot(xv1,yv1,"ro");
    plt::show();

}
