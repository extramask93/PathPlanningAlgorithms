set(SOURCE_FILES CatchMain.cpp PlannerTests.cpp LocationTests.cpp GridMapTests.cpp)
add_executable(planningtests ${SOURCE_FILES})
find_package(Python3 COMPONENTS Development NumPy)
include_directories(PotentialFields_tst_run PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(planningtests
        PRIVATE
            Python3::Python Python3::NumPy
            utils
            potentialfields
            prm
        )
