add_library(rrt "")
add_library(planning::rrt ALIAS rrt)
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
        PUBLIC
        utils
        iplanner
        )
target_link_libraries(rrt
        PUBLIC
        sampling_base)
install(TARGETS rrt EXPORT planning-export DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CMAKE_CURRENT_LIST_DIR}/RrtPlanner.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
