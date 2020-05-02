//
// Created by damian on 30.04.2020.
//

#ifndef PLANNING_VERTEX_H
#define PLANNING_VERTEX_H

#include <vector>
#include <cmath>
#include <utility>
#include "Point.h"
namespace util {
        using WorldCoordinateType = double;
        using CostType = WorldCoordinateType;
        using Path = std::vector<Point>;
        class Vertex
        {
        public:
            Vertex(util::Point coordinates, int index = -1, int parent_index = -1, CostType cost=0.0);
            void setX(WorldCoordinateType x);
            void setY(WorldCoordinateType y);
            void setIndex(int index);
            void setParentIndex(int index);
            void setCost(CostType cost);
            void setLocation(Point location);
            void addToPath(Point point);
            void cleanPath();
            int getIndex() const;
            int getParentIndex() const;
            CostType getCost() const;
            const Path& getPath() const;
            WorldCoordinateType getX() const;
            WorldCoordinateType getY() const;
            Point getLocation() const;
            WorldCoordinateType getYaw() ;
            void setYaw(WorldCoordinateType yaw);
            static const int NO_PARENT = -1;
            static const int NO_ID = -1;
        private:
            int index_;
            int parent_index_;
            WorldCoordinateType x_;
            WorldCoordinateType y_;
            CostType cost_;
            Path path_;
            WorldCoordinateType yaw_;
        };

}


#endif //PLANNING_VERTEX_H
