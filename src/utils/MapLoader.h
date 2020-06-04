//
// Created by damian on 04.05.2020.
//

#ifndef PLANNING_MAPLOADER_H
#define PLANNING_MAPLOADER_H
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <tuple>
#include <GridMap.h>
namespace util {
class MapLoader
{
  public:
    static std::tuple<std::vector<unsigned char>, int, int> loadMap(const std::string &path);
    static util::GridMap<unsigned char> loadPGMMap(const std::string &path);
  private:
    static std::vector<std::string> split(const std::string &s, char delim);
};
}// namespace util


#endif//PLANNING_MAPLOADER_H
