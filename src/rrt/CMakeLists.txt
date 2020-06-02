add_library(rrt "")
target_sources(rrt
        PRIVATE
        RrtPlanner.cpp
        PUBLIC
        #${CMAKE_CURRENT_LIST_DIR}/RrtPlanner.h
        )
target_include_directories(rrt
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )
target_link_libraries(rrt
        PRIVATE
        utils
        iplanner
        )

install(TARGETS rrt EXPORT planners-export DESTINATION lib)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/RrtPlanner.h DESTINATION include)
