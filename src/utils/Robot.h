//
// Created by damian on 25.04.2020.
//

#ifndef UTILS_ROBOT_H
#define UTILS_ROBOT_H
#include <vector>
#include <Location.h>
namespace util {
class Robot
{
public:
  static std::vector<util::Location> getMotionModel()
  {
    return std::vector<util::Location>{
      { 0, -1 }, /*|*/
      { -1, 0 }, /*-*/
      { 0, 1 }, /*|*/
      { 1, 0 } /*-*/
    };
  }
  static std::vector<char> getMotionModelArrows()
  {
    return std::vector<char>{ '^', '<', 'v', '>' };
  }
};

}// namespace util


#endif//UTILS_ROBOT_H
