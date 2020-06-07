add_library(astar "")

add_library(planning::astar ALIAS astar)
target_sources(astar
        PRIVATE
        AStar.cpp
        PUBLIC
        #${CMAKE_CURRENT_LIST_DIR}/AStar.h
        )
target_include_directories(astar
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )
target_link_libraries(astar PRIVATE iplanner project_warnings)
target_link_libraries(astar PRIVATE utils)

install(TARGETS astar EXPORT planning-export DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CMAKE_CURRENT_LIST_DIR}/AStar.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
