//
// Created by damian on 26.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_POINT_H
#define POTENTIALFIELDSPROJECT_POINT_H
#include <utility>
#include <ostream>
#include <boost/format.hpp>
namespace util {
    struct Point {
        double x = 0;
        double y = 0;

        Point(double xx = 0, double yy = 0) : x(xx), y(yy) {}

        operator std::pair<double, double>() const {
            return std::make_pair(x, y);
        }
        operator std::vector<double>() const {
            return std::vector<double>{x, y};
        }
        friend util::Point operator+(const Point &lhs, const Point &rhs) {
            return util::Point{lhs.x + rhs.x, lhs.y + rhs.y};
        }

        friend util::Point operator-(const Point &lhs, const Point &rhs) {
            return util::Point{lhs.x - rhs.x, lhs.y - rhs.y};
        }

        friend bool operator==(const Point &lhs, const Point &rhs) {
            return (lhs.x == rhs.x) && (lhs.y == rhs.y);
        }

        friend std::ostream &operator<<(std::ostream &out, const util::Point &location) {
            auto format = boost::format("(%1%,%2%)") % location.x % location.y;
            out << format.str();
            return out;
        }
    };
}
#endif //POTENTIALFIELDSPROJECT_POINT_H
