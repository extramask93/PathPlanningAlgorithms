add_library(prm "")
target_sources(prm
        PRIVATE
        PRM.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/PRM.h
        )
target_include_directories(prm
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
#external dependencies search and include
find_package(libnabo REQUIRED)
include_directories(${libnabo_INCLUDE_DIRS})

find_package (Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})
#---
target_link_libraries(prm
        PRIVATE
            utils
            iplanner
            ${libnabo_LIBRARIES}
        )

