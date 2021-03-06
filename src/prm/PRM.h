//
// Created by damian on 26.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_PRM_H
#define POTENTIALFIELDSPROJECT_PRM_H
#include "GridMap.h"
#include <Point.h>
#include <Node.h>
#include <nabo/nabo.h>
#include <queue>
#include <map>
#include <IPlanner.h>
namespace prm {
    class Prm : IPlanner{
    public:

        Prm(const util::GridMap<unsigned char> &map, unsigned int nrOfSamples);
        std::vector<std::vector<int>> generateRoadMap(unsigned nrOfSamples);
        std::vector<util::Point> generateSamples(unsigned nrOfSamples);
        std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal) override ;
        void initialize(const util::GridMap<unsigned char> &map, const util::Options &options) override;
        std::vector<util::Point> samples_;
    private:
        std::vector<util::Point> nearestNeighbors(util::Point point ,std::vector<util::Point> samples, int nNeighbors);
        void addToRoadmap(const util::Point &start, const util::Point &goal);
        bool isCollision(const util::Point &from, const util::Point &to) const;

      private:
        static constexpr double MAX_EDGE_LENGTH_ = 100;
        static constexpr int MAX_NR_OF_EDGES_PER_POINT = 15; //Lavalle, planning algorithms
        unsigned nrOfSamples_;
        util::GridMap<unsigned char> ogm_;
        util::Point start_;
        util::Point goal_;
        Nabo::NNSearchF *nns_ = nullptr;
        Eigen::MatrixXf M_;
        std::vector<std::vector<int>> edges_;
        std::map<int, util::Node<double>> openList_;
        std::map<int, util::Node<double>> closedList_;
    };
}


#endif //POTENTIALFIELDSPROJECT_PRM_H
