//
// Created by damian on 12.05.2020.
//

#ifndef PLANNING_MAPGENERATOR_H
#define PLANNING_MAPGENERATOR_H
#include <vector>

class MapGenerator
{
  public:
    static std::vector<unsigned char> generate(int size, float obstacleRatio);
};


#endif//PLANNING_MAPGENERATOR_H
