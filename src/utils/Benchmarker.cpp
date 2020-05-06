//
// Created by damian on 04.05.2020.
//

#include "Benchmarker.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <numeric>
namespace util {

void Benchmarker::start()
{
    startClock_ = std::clock();
}
BenchmarkResult Benchmarker::stop(const std::vector<util::Point> &path_)
{
    stopClock_ = std::clock();
    BenchmarkResult result;
    result.cputimems = 1000.0 * (stopClock_ - startClock_) / CLOCKS_PER_SEC;
    result.cputimes = (stopClock_ - startClock_) / CLOCKS_PER_SEC;
    double pathLength = 0.0;
    if (path_.empty()) {
        std::cout<<"Path not found\n";
    } else {
        for (int i = 0; i < path_.size() - 1; i++) {
        pathLength += std::sqrt(std::pow(path_[i].x - path_[i+1].x,2) + std::pow(path_[i].y - path_[i-1].y,2));
        }
        result.pathLength = pathLength;
        result.reachedGoal = true;
    }
    results_.push_back(result);
    return result;
}
void Benchmarker::save(const std::string &file)
{
    std::ofstream saveFile(file, std::ios::trunc);
    if(!saveFile.is_open()) {
        throw std::runtime_error("Cant open file: " + file);
    }
    saveFile<<"# reachedGoal[1/0], pathLen[m], time[ms], time[s], turns\n";
    for(const auto &result: results_) {
        saveFile << result<<'\n';
    }
}
void Benchmarker::printAverages()
{
    double sumOfPaths =
        std::accumulate(results_.begin(),results_.end(),0.0, [](const auto &a, const auto &b){return a + b.pathLength;});
    double sumOfTimes =
        std::accumulate(results_.begin(),results_.end(),0.0, [](const auto &a, const auto &b){return a + b.cputimems;});
    std::cout<<"Average path length = "<< sumOfPaths/results_.size()<<" m\n";
    std::cout<<"Average time = "<< sumOfTimes/results_.size()<<" ms\n";
}
}// namespace util