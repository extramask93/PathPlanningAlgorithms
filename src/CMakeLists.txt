add_executable(runner main.cpp)

add_subdirectory(potentialfields)
add_subdirectory(prm)
add_subdirectory(rrtstar)
add_subdirectory(rrt)
add_subdirectory(utils)

find_package(Python3 COMPONENTS Development NumPy)
include_directories(runner PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

target_link_libraries(runner
        PRIVATE
        Python3::Python Python3::NumPy
        potentialfields
        prm
        utils
        rrtstar
        rrt
        )