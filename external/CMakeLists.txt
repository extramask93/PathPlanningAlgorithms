include(FetchContent)

FetchContent_Declare(
        nabo
        GIT_REPOSITORY https://github.com/ethz-asl/libnabo.git
        GIT_TAG        d4f45e2b2f9b811344cd5002346b9fb52ac2520e
)
FetchContent_GetProperties(nabo)
if(NOT nabo_POPULATED)
    FetchContent_Populate(nabo)
    add_subdirectory(${nabo_SOURCE_DIR} ${nabo_BINARY_DIR})
endif()

