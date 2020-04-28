add_executable(runner main.cpp)

add_subdirectory(potentialfields)
add_subdirectory(prm)
add_subdirectory(utils)
target_link_libraries(runner
        PRIVATE
        potentialfields
        prm
        utils
        )