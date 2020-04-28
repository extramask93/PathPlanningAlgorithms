#find_package( Boost 1.67 )
#include_directories( ${Boost_INCLUDE_DIR} )
add_library(potentialfields "")
target_sources(potentialfields
        PRIVATE
            PotentialFieldsPlanner.cpp
        PUBLIC
            ${CMAKE_CURRENT_LIST_DIR}/PotentialFieldsPlanner.h
        )
target_include_directories(potentialfields
        PUBLIC
            ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(potentialfields utils)