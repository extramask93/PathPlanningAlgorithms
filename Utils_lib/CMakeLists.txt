project(Utils)
set(HEADER_FILES GridMap.h Robot.h Location.h MapException.h)
set(SOURCE_FILES GridMap.cpp Robot.cpp )
find_package( Boost 1.67 )
include_directories( ${Boost_INCLUDE_DIR} )

add_library(Utils_lib STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(Utils_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})