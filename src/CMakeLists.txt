add_executable(runner main.cpp)

#find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(runner PUBLIC ${PLOT_INCLUDE_DIR} SYSTEM)
add_dependencies(runner MatplotlibcppDownload)

add_subdirectory(utils)
add_subdirectory(IPlanner)
add_subdirectory(potentialfields)
add_subdirectory(prm)
add_subdirectory(ga)
add_subdirectory(rrtstar)
add_subdirectory(rrt)
add_subdirectory(astar)
add_subdirectory(ants)
add_subdirectory(depthfirst)

target_link_libraries(runner
        PRIVATE
        potentialfields
        prm
        utils
        rrtstar
        rrt
        astar
        ants
        depthfirst
        project_options
        #Python3::Python Python3::NumPy
        )
