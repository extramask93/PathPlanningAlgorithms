//
// Created by damian on 12.05.2020.
//

#include "MapGenerator.h"
#include <cmath>
#include <random>
std::vector<unsigned char> MapGenerator::generate(int size, float obstacleRatio)
{
    auto vc = std::vector<unsigned char>(size*size,1);
    int nrOfObstacles = floor(obstacleRatio*vc.size());
    static std::mt19937 randomGenerator(std::random_device{}());
    using Distribution = std::uniform_int_distribution<>;
    static Distribution distribution(0,vc.size()-1);
    for(int i = 0; i < nrOfObstacles; i++) {
        vc[distribution(randomGenerator)] = 0;
    }
    return vc;
}
