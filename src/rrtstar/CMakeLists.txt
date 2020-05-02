add_library(rrtstar "")
target_sources(rrtstar
        PRIVATE
        rrtstar.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/rrtstar.h
        )
target_include_directories(rrtstar
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(rrtstar
        PRIVATE
        utils
        )