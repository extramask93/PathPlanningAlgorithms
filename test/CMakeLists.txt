set(SOURCE_FILES CatchMain.cpp PlannerTests.cpp LocationTests.cpp GridMapTests.cpp rrtstarTests.cpp GridPlannersTests.cpp
        SamplingMethodsTests.cpp HeuristicPlannersTest.cpp)
add_executable(planningtests ${SOURCE_FILES})
target_link_libraries(planningtests
        PRIVATE
            utils
            potentialfields
            prm
            rrtstar
            rrt
            astar
            depthfirst
            ants
        )
