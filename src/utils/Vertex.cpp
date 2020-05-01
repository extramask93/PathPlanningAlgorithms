//
// Created by damian on 30.04.2020.
//

#include "Vertex.h"
namespace util {
    Vertex::Vertex(util::Point coordinate, int index,
                           int parent_index, CostType cost)
            : index_(index), parent_index_(parent_index),x_(coordinate.x),y_(coordinate.y),cost_(cost),
              path_(std::vector<Point>{}),yaw_(0)

    {

    }

    void Vertex::setX(WorldCoordinateType x) {
        x_ = x;
    }

    void Vertex::setY(WorldCoordinateType y) {
        y_=y;
    }

    void Vertex::setIndex(int index) {
        index_ = index;
    }

    void Vertex::setParentIndex(int index) {
        parent_index_ = index;
    }

    void Vertex::setCost(CostType cost) {
        cost_ = cost;
    }

    int Vertex::getIndex() const {
        return index_;
    }

    int Vertex::getParentIndex() const {
        return parent_index_;
    }

    CostType Vertex::getCost() const {
        return cost_;
    }

    WorldCoordinateType Vertex::getX() const {
        return x_;
    }

    WorldCoordinateType Vertex::getY() const {
        return y_;
    }

    void Vertex::setLocation(Point location) {
        setX(location.x);
        setY(location.y);
    }

    Point Vertex::getLocation() const {
        return Point(x_,y_);
    }

    void Vertex::addToPath(Point point) {
        path_.push_back({point.x, point.y});
    }

    void Vertex::cleanPath() {
        path_.clear();
    }

    const Path& Vertex::getPath() const {
        return path_;
    }

    WorldCoordinateType Vertex::getYaw() {
        return yaw_;
    }

    void Vertex::setYaw(WorldCoordinateType yaw) {
        yaw_ = yaw;
    }

}
