add_library(iplanner "")
add_library(planning::iplanner ALIAS iplanner)
target_sources(iplanner
        PRIVATE
        IPlanner.cpp
        PUBLIC
 #       ${CMAKE_CURRENT_LIST_DIR}/IPlanner.h
        )
target_include_directories(iplanner
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>

        )
target_link_libraries(iplanner PRIVATE utils project_warnings)

install(TARGETS iplanner EXPORT planning-export DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CMAKE_CURRENT_LIST_DIR}/IPlanner.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
