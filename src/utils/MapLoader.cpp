//
// Created by damian on 04.05.2020.
//

#include "MapLoader.h"
namespace util {

std::vector<std::string> MapLoader::split(const std::string &s, char delim)
{
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (std::getline(ss, item, delim)) {
        result.push_back(item);
    }

    return result;
}
std::tuple<std::vector<unsigned char>, int, int> MapLoader::loadMap(const std::string &path)
{
    std::vector<unsigned char> map;
    std::fstream file;
    std::string line;
    file.open(path, std::fstream::in);
    int width = 0;
    int height = 0;
    while (std::getline(file, line, '\n')) {
        height++;
        auto tokens = split(line, ',');
        for (auto &token : tokens) {
            if (height == 1) {
                width++;
            }
            int value = false;
            std::stringstream(token) >> value;
            map.push_back(value);
        }
    }
    return std::make_tuple(map, width, height);
}
}
