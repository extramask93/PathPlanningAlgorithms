//
// Created by damian on 19.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_POTENTIALFIELDSPLANNER_H
#define POTENTIALFIELDSPROJECT_POTENTIALFIELDSPLANNER_H
#include <vector>
#include <boost/optional.hpp>
namespace pf {
    using uint = unsigned int;
    class PotentialFieldsPlanner {
    public:
        PotentialFieldsPlanner(const std::vector<bool> &ogm, uint width, uint height, double resolution = 1.0);
        std::vector<uint> makePlan(uint startIndex, uint goalIndex);
        double calculateAttractivePotential(uint currentCelIndex, uint goalIndex, double KD = 3.0);
        double calculateRepulsivePotential(uint currentIndex);
        void calculatePotentialField(uint index);
        void printMapOfPotentials();
        void printMap();
        void printPath();
        boost::optional<std::pair<int, int>> indexToMap(unsigned int index);
        uint mapToIndex(std::pair<int,int> location);
        bool isWithinMapBounds(std::pair<int,int> location);
    private:
        void findObstacles();
        boost::optional<uint> findClosestObstacleIndex(uint index);
        std::vector<std::pair<int,int>> getMotionModel() const;
        double distance(uint fromIndex, uint toIndex);
    private:
        std::vector<bool> ogm_;
        std::vector<uint> obstacleIndexes_;
        std::vector<double> potentialMap_;
        std::vector<std::pair<uint,char>> movePath_;
        uint goalIndex_;
        uint startIndex_;
        uint mapWidth_;
        uint mapHeight_;
        uint mapResolution_;
        static constexpr bool OBSTACLE = 1;
        static constexpr bool FREE = 0;
        static constexpr double ATTRACTIVE_POTENTIAL_GAIN = 5.0;
        static constexpr double REPULSIVE_POTENTIAL_GAIN = 100.0;
        static constexpr double POTENTIAL_AREA_WIDTH = 30.0;
    };
}


#endif //POTENTIALFIELDSPROJECT_POTENTIALFIELDSPLANNER_H
