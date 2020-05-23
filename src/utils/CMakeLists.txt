add_library(utils "")

#find_package(Python3 COMPONENTS Development NumPy)
#include_directories(utils PUBLIC ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
#target_link_libraries(utils
#        PRIVATE
#        CONAN_PKG::boost
#        )
find_package(Boost 1.36.0)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()
target_sources(utils
        PRIVATE
        GridMap.cpp
        Options.cpp
        Robot.cpp
        Vertex.cpp
        Benchmarker.cpp
        MapLoader.cpp
        MapGenerator.cpp
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
        ${CMAKE_CURRENT_LIST_DIR}/Options.h
 #       ${CMAKE_CURRENT_LIST_DIR}/matplotlibcpp.h
        ${CMAKE_CURRENT_LIST_DIR}/MapGenerator.h
        )
target_include_directories(utils
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
