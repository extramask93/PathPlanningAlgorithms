add_executable(runner main.cpp)



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
add_subdirectory(sampling_base)

target_link_libraries(runner
        PRIVATE
        potentialfields
        sampling_base
        prm
        utils
        rrtstar
        rrt
        astar
        ants
        depthfirst
        project_options
        )
