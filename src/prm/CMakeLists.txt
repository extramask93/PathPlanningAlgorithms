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
        )

find_package(libnabo REQUIRED)
include_directories(${libnabo_INCLUDE_DIRS})
message(STATUS "++++++++++++++++++++++++++++++++++" ${libnabo_INCLUDE_DIRS})
message(STATUS "++++++++++++++++++++++++++++++++++" ${libnabo_LIBRARIES})
target_link_libraries(prm
        PRIVATE
            utils
            iplanner
            CONAN_PKG::eigen
            ${libnabo_LIBRARIES}
        )

