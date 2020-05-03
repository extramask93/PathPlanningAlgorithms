//
// Created by damian on 26.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_PRM_H
#define POTENTIALFIELDSPROJECT_PRM_H
#include "GridMap.h"
#include "nabo/nabo.h"
#include <Point.h>
#include <Node.h>
#include <queue>
#include <map>

namespace prm {
    class Prm {
    public:

        Prm(const util::GridMap<int> &map, unsigned int nrOfSamples);
        std::vector<std::vector<int>> generateRoadMap(unsigned nrOfSamples);
        std::vector<util::Point> generateSamples(unsigned nrOfSamples);
        std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal);
        std::vector<util::Point> samples_;
    private:
        std::vector<util::Point> nearestNeighbors(util::Point point ,std::vector<util::Point> samples, int nNeighbors);
        bool isCollision(const util::Point &from, const util::Point &to) const;
    private:
        static constexpr double MAX_EDGE_LENGTH_ = 100;
        static constexpr int MAX_NR_OF_EDGES_PER_POINT = 5;
        unsigned nrOfSamples_;
        util::GridMap<int> ogm_;
        util::Point start_;
        util::Point goal_;
        std::map<int, util::Node<double>> openList_;
        std::map<int, util::Node<double>> closedList_;
    };
}


#endif //POTENTIALFIELDSPROJECT_PRM_H
