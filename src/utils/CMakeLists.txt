add_library(utils "")
target_sources(utils
        PRIVATE
        GridMap.cpp
        Robot.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Location.h
        ${CMAKE_CURRENT_LIST_DIR}/MapException.h
        ${CMAKE_CURRENT_LIST_DIR}/MapIterator.h
        ${CMAKE_CURRENT_LIST_DIR}/Node.h
        ${CMAKE_CURRENT_LIST_DIR}/Point.h
        ${CMAKE_CURRENT_LIST_DIR}/RoadPoint.h
        ${CMAKE_CURRENT_LIST_DIR}/Robot.h
        )
target_include_directories(utils
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )