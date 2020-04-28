cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(Planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
add_subdirectory(external)
add_subdirectory(src)
add_subdirectory(test)
#add run files
#set(SOURCE_FILES main.cpp)
#add_executable(PotentialFields_run ${SOURCE_FILES})

#list(APPEND flags "-Wextra" "-Wall" "-Wpedantic")
#target_compile_options(PotentialFields_run PRIVATE ${flags})
#set_target_properties(PotentialFields_run PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF CXX_STANDARD_REQUIRED ON)
#add subprojects
#add_subdirectory(PotentialFields_lib)
#add_subdirectory(Utils_lib)
#add_subdirectory(PotentialFields_tst)
#add_subdirectory(external/upstream)

#include_directories(PotentialFields_lib Utils_lib)
#target_link_libraries(PotentialFields_run PotentialFields_lib Utils_lib KDTree)
#target_link_libraries(PotentialFields_lib Utils_lib)

