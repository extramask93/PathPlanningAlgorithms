include(FetchContent)

FetchContent_Declare(
        nabo
        GIT_REPOSITORY https://github.com/extramask93/libnabo
)
FetchContent_GetProperties(nabo)
if(NOT nabo_POPULATED)
    FetchContent_Populate(nabo)
    add_subdirectory(${nabo_SOURCE_DIR} ${nabo_BINARY_DIR})
endif()
include(ExternalProject)
ExternalProject_Add(MatplotlibcppDownload
        PREFIX 3rd_party
        #--Download step--------------
        GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
        GIT_TAG master
        #--Configure step-------------
        CONFIGURE_COMMAND ""
        #--Build step-----------------
        BUILD_COMMAND ""
        #--Install step---------------
        UPDATE_COMMAND "" # Skip annoying updates for every build
        INSTALL_COMMAND ""
        )

SET(PLOT_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/MatplotlibcppDownload)
