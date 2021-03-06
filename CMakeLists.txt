cmake_minimum_required(VERSION 3.15)
project(planning LANGUAGES CXX)

include(cmake/StandardProjectSettings.cmake)
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_14)

add_library(project_warnings INTERFACE)
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)


include(cmake/StaticAnalyzers.cmake)
include(cmake/Doxygen.cmake)
enable_doxygen()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

option(ENABLE_TESTING "Enable Test Builds" ON)
option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)

# Set up some extra Conan dependencies based on our needs
# before loading Conan
set(CONAN_EXTRA_REQUIRES "")
set(CONAN_EXTRA_OPTIONS "")

include(cmake/Conan.cmake)
run_conan()
include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
find_package (Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
message(STATUS "-----------------------" ${Eigen3_INCLUDE_DIRS})
#

if(ENABLE_TESTING)
    enable_testing()
    message("Building Tests...")
    add_subdirectory(test)
endif()
include(external/CMakeLists.txt)
add_subdirectory(src)
add_subdirectory(resources)

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
install(EXPORT planning-export
        FILE
        planningConfig.cmake
        NAMESPACE
        planning::
        DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/planning
        )

include(CMakePackageConfigHelpers)
#configure_package_config_file(
#        "planningConfig.cmake.in"
#        "planningConfig.cmake"
#        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planning
#        PATH_VARS
#        CMAKE_INSTALL_LIBDIR
#)

write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/planningConfigVersion.cmake
        VERSION 1.0.0
        COMPATIBILITY SameMajorVersion
)

### Install Config and ConfigVersion files
install(
        #do not use custom config we've got automatic generation by install(export
#        FILES "${CMAKE_CURRENT_BINARY_DIR}/planningConfig.cmake"
        FILES "${CMAKE_CURRENT_BINARY_DIR}/planningConfigVersion.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/planning"
)
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE planning)
