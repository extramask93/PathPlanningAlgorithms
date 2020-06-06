add_library(utils "")
add_library(planning::Utils ALIAS utils)

option(BUILD_WITH_PLOT "Include matplotlib into build" OFF)
find_package(Python3 COMPONENTS Development NumPy)

if((Python3_FOUND) AND (BUILD_WITH_PLOT))
    message("--------Building with matplotlib support-----------")
    add_dependencies(utils MatplotlibcppDownload)
    target_compile_definitions(utils PUBLIC MATPLOTLIB)
    target_include_directories(utils SYSTEM PUBLIC ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} )
    target_include_directories(utils SYSTEM PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PLOT_INCLUDE_DIR}>
        )
    target_link_libraries(utils INTERFACE Python3::Python Python3::NumPy)
    install(FILES ${PLOT_INCLUDE_DIR}/matplotlibcpp.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
else()
    message("--------Building without matplotlib support-----------")
endif()
set(INCLUDE_FILES

        ${CMAKE_CURRENT_LIST_DIR}/Location.h
        ${CMAKE_CURRENT_LIST_DIR}/MapException.h
        ${CMAKE_CURRENT_LIST_DIR}/MapIterator.h
        ${CMAKE_CURRENT_LIST_DIR}/GridMap.h
        ${CMAKE_CURRENT_LIST_DIR}/Node.h
        ${CMAKE_CURRENT_LIST_DIR}/Point.h
        ${CMAKE_CURRENT_LIST_DIR}/RoadPoint.h
        ${CMAKE_CURRENT_LIST_DIR}/Robot.h
        ${CMAKE_CURRENT_LIST_DIR}/Vertex.h
        ${CMAKE_CURRENT_LIST_DIR}/Edge.h
        ${CMAKE_CURRENT_LIST_DIR}/Benchmarker.h
        ${CMAKE_CURRENT_LIST_DIR}/MapLoader.h
        ${CMAKE_CURRENT_LIST_DIR}/Options.h
        ${CMAKE_CURRENT_LIST_DIR}/MapGenerator.h
        )
target_sources(utils
        PRIVATE
        GridMap.cpp
        Options.cpp
        Robot.cpp
        Vertex.cpp
        Benchmarker.cpp
        MapLoader.cpp
        MapGenerator.cpp
        )
target_include_directories(utils
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )
install(TARGETS utils EXPORT planning-export DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${INCLUDE_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
