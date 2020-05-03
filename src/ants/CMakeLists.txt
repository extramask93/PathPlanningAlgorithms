add_library(ants "")
target_sources(ants
        PRIVATE
        AntColony.cpp
        Ant.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/AntColony.h
        ${CMAKE_CURRENT_LIST_DIR}/Ant.h
        )
#------------------------------------
find_package (Boost 1.36 REQUIRED)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message("Boost libraries not found")
endif()
#-----------------------------------
target_include_directories(ants
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(ants utils)