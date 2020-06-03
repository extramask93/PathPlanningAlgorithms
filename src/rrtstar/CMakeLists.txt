add_library(rrtstar "")
add_library(planning::rrtstar ALIAS rrtstar)
target_sources(rrtstar
        PRIVATE
        rrtstar.cpp
        PUBLIC
        #${CMAKE_CURRENT_LIST_DIR}/rrtstar.h
        )
target_include_directories(rrtstar
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )
target_link_libraries(rrtstar
        PRIVATE
        utils
        iplanner
        )

install(TARGETS rrtstar EXPORT planning-export DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CMAKE_CURRENT_LIST_DIR}/rrtstar.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

