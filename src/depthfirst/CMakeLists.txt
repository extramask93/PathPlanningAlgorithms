add_library(depthfirst "")
target_sources(depthfirst
        PRIVATE
        DepthFirst.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/DepthFirst.h
        )
target_include_directories(depthfirst
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(depthfirst utils iplanner)
