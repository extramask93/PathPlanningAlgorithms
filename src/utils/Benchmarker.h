//
// Created by damian on 04.05.2020.
//

#ifndef PLANNING_BENCHMARKER_H
#define PLANNING_BENCHMARKER_H

#include <vector>
#include <list>
#include <iomanip>
#include "Point.h"
namespace util {
struct BenchmarkResult {
    bool reachedGoal = 0;
    double pathLength = 0.0;
    double cputimems = 0.0;
    double cputimes = 0.0;
    int nrOfTurns =0;
    friend std::ostream& operator<<(std::ostream &os, const BenchmarkResult &result) {
        os<< std::setprecision(4)<<result.reachedGoal<<" "<<result.pathLength<<" "<<result.cputimems <<" "<<result.cputimes;
        os<<" "<<result.nrOfTurns;
        return os;
    }
};
class Benchmarker
{
  public:
    Benchmarker() = default;
    void start();
    BenchmarkResult stop(const std::vector<util::Point> &path_);
    void printAverages();
    void save(const std::string &fileLoc);
    void save(int nr, const std::string &file);

  private:
    std::clock_t startClock_;
    std::clock_t stopClock_;
    std::list<util::BenchmarkResult> results_;
};
}


#endif//PLANNING_BENCHMARKER_H
