//
// Created by damian on 19.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_POTENTIALFIELDSPLANNER_H
#define POTENTIALFIELDSPROJECT_POTENTIALFIELDSPLANNER_H
#include <vector>
#include <boost/optional.hpp>
#include <GridMap.h>
#include <Robot.h>

namespace pf {
    class PotentialFieldsPlanner {
    public:
        explicit PotentialFieldsPlanner(const util::GridMap<unsigned char> &ogm);
        std::vector<util::Point> makePlan(const util::Point &start, const util::Point &goal);
        double calculateAttractivePotential(const util::Location &currentLocation, const util::Location &goalIndex) const;
        double calculateRepulsivePotential(const util::Location &currentLocation) const;
        void calculatePotentialField(const util::Location &location);
        util::GridMap<double> potentialMap_;
    private:
        util::GridMap<unsigned char> ogm_;
        util::Location goal_;
        util::Location start_;
        static constexpr bool OBSTACLE = 1;
        static constexpr bool FREE = 0;
        static constexpr double ATTRACTIVE_POTENTIAL_GAIN = 5.0;
        static constexpr double REPULSIVE_POTENTIAL_GAIN = 100.0;
        static constexpr double POTENTIAL_AREA_WIDTH = 30.0;
    };
}


#endif //POTENTIALFIELDSPROJECT_POTENTIALFIELDSPLANNER_H
