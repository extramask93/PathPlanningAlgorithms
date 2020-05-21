add_library(prm "")
target_sources(prm
        PRIVATE
        PRM.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/PRM.h
        )


target_include_directories(prm
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${nabo_SOURCE_DIR}
        )

target_link_libraries(prm
        PRIVATE
        utils
        iplanner
        CONAN_PKG::eigen
        nabo
        )
