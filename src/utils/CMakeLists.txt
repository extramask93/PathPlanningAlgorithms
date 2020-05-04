add_library(utils "")

#find_package(Python2 COMPONENTS Development NumPy)
#include_directories(utils PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
#target_link_libraries(utils
#        PRIVATE
#        Python2::Python Python2::NumPy
#        )

target_sources(utils
        PRIVATE
        GridMap.cpp
        Robot.cpp
        Vertex.cpp
        Benchmarker.cpp
        MapLoader.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Location.h
        ${CMAKE_CURRENT_LIST_DIR}/MapException.h
        ${CMAKE_CURRENT_LIST_DIR}/MapIterator.h
        ${CMAKE_CURRENT_LIST_DIR}/Node.h
        ${CMAKE_CURRENT_LIST_DIR}/Point.h
        ${CMAKE_CURRENT_LIST_DIR}/RoadPoint.h
        ${CMAKE_CURRENT_LIST_DIR}/Robot.h
        ${CMAKE_CURRENT_LIST_DIR}/Vertex.h
        ${CMAKE_CURRENT_LIST_DIR}/Edge.h
        ${CMAKE_CURRENT_LIST_DIR}/Benchmarker.h
        ${CMAKE_CURRENT_LIST_DIR}/MapLoader.h
        )
target_include_directories(utils
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
