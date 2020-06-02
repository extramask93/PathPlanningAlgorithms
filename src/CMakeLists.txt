add_executable(runner main.cpp)

find_package(Python3 COMPONENTS Development NumPy)
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
        Python3::Python Python3::NumPy
        )
install(TARGETS runner DESTINATION bin)

install(EXPORT planners-export
        FILE
        utils.cmake
        NAMESPACE
        Planners::
        DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/Planners
        )

include(CMakePackageConfigHelpers)
configure_package_config_file(
        "Config.cmake.in"
        "Planners.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Planners
        PATH_VARS
        CMAKE_INSTALL_LIBDIR
)

write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/PlannersConfigVersion.cmake
        VERSION 1.0.0
        COMPATIBILITY SameMajorVersion
)

### Install Config and ConfigVersion files
install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/Planners.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/PlannersConfigVersion.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Planners"
)
export(PACKAGE Planners)
