# automatically enable catch2 to generate ctest targets
if(CONAN_CATCH2_ROOT_DEBUG)
    include(${CONAN_CATCH2_ROOT_DEBUG}/lib/cmake/Catch2/Catch.cmake)
else()
    include(${CONAN_CATCH2_ROOT}/lib/cmake/Catch2/Catch.cmake)
endif()


add_library(catch_main STATIC CatchMain.cpp)
target_link_libraries(catch_main PUBLIC CONAN_PKG::catch2)
target_link_libraries(catch_main PRIVATE project_options)


set(SOURCE_FILES PlannerTests.cpp GridMapTests.cpp rrtstarTests.cpp GridPlannersTests.cpp
        SamplingMethodsTests.cpp HeuristicPlannersTest.cpp MapLoaderTests.cpp)

add_executable(tests ${SOURCE_FILES})
target_link_libraries(tests PRIVATE
        #project_warnings
        project_options
        catch_main
        utils
        potentialfields
        prm
        rrtstar
        rrt
        astar
        depthfirst
        ants)

catch_discover_tests(
        tests
        TEST_PREFIX
        "speedtests."
        EXTRA_ARGS
        -s
        --reporter=xml
        --out=tests.xml)
######################################################################################333

add_executable(tests_fast LocationTests.cpp)
target_link_libraries(
        tests_fast PRIVATE project_options project_warnings utils
        catch_main)
catch_discover_tests(
        tests_fast
        TEST_PREFIX
        "unittests."
        EXTRA_ARGS
        -s
        --reporter=xml
        --out=tests_fast.xml)
