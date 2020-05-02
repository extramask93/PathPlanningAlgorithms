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

namespace plt = matplotlibcpp;
std::vector<std::string> split (const std::string &s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while (std::getline (ss, item, delim)) {
        result.push_back (item);
    }

    return result;
}
std::tuple<std::vector<uint8_t>,int,int> loadMap() {
    std::vector<uint8_t> map;
    std::fstream file;
    std::string line;
    file.open("/home/damian/Planning/map.csv", std::fstream::in);
    int width = 0;
    int height = 0;
    while(std::getline(file,line,'\n')) {
        height++;
        auto tokens = split(line,',');
        for(auto &token : tokens) {
            if(height ==1) {
                width++;
            }
            uint8_t value = false;
            std::stringstream (token) >> value;
            map.push_back(value);
        }
    }
    return std::make_tuple(map,width,height);
}
TEST_CASE( "One should be able to create prm planner with ogm supplied", "[Planner, PRM]" ) {
    std::vector<int> ogm {0, 0, 0, 0, 0,0,0,
                          0, 0, 0, 0, 0,0,0,
                          0, 0, 0, 0, 0,0,0,
                          0, 0, 1, 1, 0,0,0,
                          0, 0, 1, 1, 0,0,0,
                          0, 0, 0, 0, 0,0,0,
                          0, 0, 0, 0, 0,0,0};
    util::GridMap<int> map(ogm, 7,7,1.0);
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
    std::vector<uint8_t> ogm;
    unsigned int width =3;
    unsigned int height =2;
    auto tu = loadMap();
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
