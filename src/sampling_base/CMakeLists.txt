add_library(sampling_base "")
add_library(planning::sampling_base ALIAS sampling_base)
target_sources(sampling_base
        PRIVATE
        SamplingBase.cpp
        PUBLIC
        )
target_include_directories(sampling_base
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )
target_link_libraries(sampling_base
        PRIVATE
        utils
        iplanner
        )

install(TARGETS sampling_base EXPORT planning-export DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CMAKE_CURRENT_LIST_DIR}/SamplingBase.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
