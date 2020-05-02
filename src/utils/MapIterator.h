//
// Created by damian on 25.04.2020.
//

#ifndef POTENTIALFIELDSPROJECT_MAPITERATOR_H
#define POTENTIALFIELDSPROJECT_MAPITERATOR_H
#include <iterator>
#include <Location.h>
/*template<class CELL_T>
class MapIterator
{
private:
    util::Location location_;
    class CellHolder
    {
        CELL_T value_;
    public:
        CellHolder(CELL_T value): value_(value) {}
        CELL_T operator*() { return value_; }
    };

public:
    typedef CELL_T                     value_type;
    typedef std::ptrdiff_t          difference_type;
    typedef int*                    pointer;
    typedef int&                    reference;
    typedef std::input_iterator_tag iterator_category;

    explicit MapIterator(util::Location location) : location_(location) {}
    int operator*() const { return location_; }
    bool operator==(const MapIterator<CELL_T>& other) const { return value_ == other.value_; }
    bool operator!=(const MapIterator<CELL_T>& other) const { return !(*this == other); }
    util::Location operator++(int)
    {
        util::Location ret(value_);
        ++*this;
        return ret;
    }
    MapIterator<CELL_T>& operator++()
    {
        ++value_;
        return *this;
    }
};
 */
#endif //POTENTIALFIELDSPROJECT_MAPITERATOR_H
