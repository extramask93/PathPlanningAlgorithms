add_library(rrtstar "")
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

install(TARGETS rrtstar EXPORT planners-export DESTINATION lib)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/rrtstar.h DESTINATION include)

