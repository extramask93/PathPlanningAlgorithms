project(PotentialFields)
set(HEADER_FILES PotentialFieldsPlanner.h)
set(SOURCE_FILES PotentialFieldsPlanner.cpp)
find_package( Boost 1.67 )
include_directories( ${Boost_INCLUDE_DIR} )

add_library(PotentialFields_lib STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(PotentialFields_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})