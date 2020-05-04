//
// Created by damian on 04.05.2020.
//

#include "Benchmarker.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <cmath>
namespace util {

Benchmarker::Benchmarker(const std::vector<util::Point> &path, const Point &start, const Point &stop) : path_(path),
                                                                                                        start_(start),
                                                                                                        stop_(stop)
{
    startClock_ = std::clock();
}
Benchmarker::~Benchmarker()
{
    stopClock_ = std::clock();
    std::cout << std::fixed << std::setprecision(2) << "CPU time used: "
              << 1000.0 * (stopClock_ - startClock_) / CLOCKS_PER_SEC << " ms\n";
    double pathLength = 0.0;
    if (path_.empty()) {
        std::cout<<"Path not found\n";
    } else {
        for (int i = 0; i < path_.size() - 1; i++) {
        pathLength += std::sqrt(std::pow(path_[i].x - path_[i+1].x,2) + std::pow(path_[i].y - path_[i-1].y,2));
        }
        std::cout<<"Path length: "<<pathLength<<"m\n";
    }
}
}// namespace util