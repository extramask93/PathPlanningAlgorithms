cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(PotentialFieldsProject LANGUAGES CXX)

#add run files
set(SOURCE_FILES main.cpp)
add_executable(PotentialFields_run ${SOURCE_FILES})

list(APPEND flags "-Wextra" "-Wall" "-Wpedantic")
target_compile_options(PotentialFields_run PRIVATE ${flags})
set_target_properties(PotentialFields_run PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF CXX_STANDARD_REQUIRED ON)
#add subprojects
add_subdirectory(PotentialFields_lib)
add_subdirectory(PotentialFields_tst)
#add dependencies
include_directories(PotentialFields_lib)
target_link_libraries(PotentialFields_run PotentialFields_lib)

