//
// Created by damian on 04.05.2020.
//

#include <iostream>
#include <cstddef>
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
util::GridMap<unsigned char>  MapLoader::loadPGMMap(const std::string &path)
{
    std::ifstream file(path);
    if(!file) {
        std::cerr<< "Error during loading file: "<<path;
    }
    /*load first line containing version*/
    std::string buffer;
    std::getline(file,buffer);
    if(buffer.compare("P2") != 0) {
        std::cerr<<"Error, only version P2 supported, but given: "<<buffer;
    }
    /*get comment out of the way*/
    std::getline(file,buffer);
    /*read payload*/
    std::stringstream stream;
    stream << file.rdbuf();
    int rows = 0;
    int columns =0;
    stream>>columns>>rows;
    int maxPixelValue = 0;
    stream>>maxPixelValue;
    std::vector<unsigned char> grid(rows*columns, 0);
    for(int row =0; row < rows; row++) {
        for(int column =0; column < columns; column++) {
            int temp;
            stream>>temp;
            grid[row*rows + column] = temp > 0 ? 0 :1 ;
        }
    }
    return util::GridMap<unsigned char>(grid,rows,columns,1.0);

}
}
