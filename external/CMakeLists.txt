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
