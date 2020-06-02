add_library(depthfirst "")
target_sources(depthfirst
        PRIVATE
        DepthFirst.cpp
        PUBLIC
 #       ${CMAKE_CURRENT_LIST_DIR}/DepthFirst.h
        )
target_include_directories(depthfirst
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )
target_link_libraries(depthfirst PRIVATE utils iplanner project_warnings)

install(TARGETS depthfirst project_warnings EXPORT planners-export DESTINATION lib)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/DepthFirst.h DESTINATION include)
