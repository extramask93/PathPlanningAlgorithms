add_library(astar "")
target_sources(astar
        PRIVATE
        AStar.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/AStar.h
        )
target_include_directories(astar
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(astar utils)