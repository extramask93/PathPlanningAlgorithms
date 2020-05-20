include(FetchContent)
FetchContent_Declare(
        projD
        GIT_REPOSITORY https://github.com/ethz-asl/libnabo.git
        GIT_TAG        d4f45e2b2f9b811344cd5002346b9fb52ac2520e
)
FetchContent_GetProperties(projD)
if(NOT projd_POPULATED)
    FetchContent_Populate(projD)
    add_subdirectory(${projd_SOURCE_DIR} ${projd_BINARY_DIR})
endif()
