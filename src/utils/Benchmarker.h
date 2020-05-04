//
// Created by damian on 04.05.2020.
//

#ifndef PLANNING_BENCHMARKER_H
#define PLANNING_BENCHMARKER_H

#include <vector>
#include "Point.h"
namespace util {
class Benchmarker
{
  public:
    Benchmarker(const std::vector<util::Point> &path, const Point &start, const Point &stop);
    virtual ~Benchmarker();

  private:
    std::clock_t startClock_;
    std::clock_t stopClock_;
    const std::vector<util::Point> &path_;
    const util::Point &start_;
    const util::Point &stop_;
};
}


#endif//PLANNING_BENCHMARKER_H
