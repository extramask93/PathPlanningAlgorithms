add_library(iplanner "")
target_sources(iplanner
        PRIVATE
        IPlanner.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/IPlanner.h
        )
target_include_directories(iplanner
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(iplanner utils)
