//
// Created by damian on 01.05.2020.
//

#ifndef PLANNING_RRTPLANNER_H
#define PLANNING_RRTPLANNER_H

#include <Vertex.h>
#include <GridMap.h>
#include <Point.h>
#include <IPlanner.h>
#include <SamplingBase.h>

namespace rrt {
using sb::SamplingBase;
class RrtPlanner : public IPlanner
    , public SamplingBase<RrtPlanner>
{
    friend class SamplingBase<RrtPlanner>;

  public:
    explicit RrtPlanner(std::shared_ptr<util::GridMap<unsigned char>> &map);
    std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal) override;
    void initialize(const util::GridMap<unsigned char> &map, const util::Options &options) override;
    void setMaxExtendDistance(double distance);
    void setMaxNrOfIterations(unsigned iterationNr);

  protected:
    int makeGraph(const util::Point &start, const util::Point &goal);

  private:
    std::vector<util::Vertex> vertexList_;
    util::Vertex goalVertex_;
    util::Vertex startVertex_;
    double goalRadius_;
    double goalSamplingRatio_;
    unsigned maxNrOfIterations_;
    double maxExtendDistance_;
};
}// namespace rrt


#endif//PLANNING_RRTPLANNER_H
