//
// Created by damian on 25.04.2020.
//

#ifndef UTILS_LOCATION_H
#define UTILS_LOCATION_H

#include <boost/format.hpp>

namespace util {
    struct Location {
        int x = 0;
        int y = 0;
        friend bool operator==(const Location& lhs, const Location& rhs){
            return (lhs.x == rhs.x) && (lhs.y == rhs.y);
        }
        friend std::ostream& operator<<(std::ostream &out, const util::Location &location) {
            auto format = boost::format ("(%1%,%2%)") % location.x % location.y;
            out<< format.str();
            return out;
        }
    };
}


#endif //UTILS_LOCATION_H
