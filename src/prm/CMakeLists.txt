add_library(prm "")
target_sources(prm
        PRIVATE
        PRM.cpp
        PUBLIC
 #       ${CMAKE_CURRENT_LIST_DIR}/PRM.h
        )


target_include_directories(prm
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )

target_include_directories(prm SYSTEM
        PUBLIC
        ${nabo_SOURCE_DIR}
        )
target_link_libraries(prm
        PRIVATE
        utils
        iplanner
        CONAN_PKG::eigen
        nabo
        #project_warnings
        )

install(TARGETS prm EXPORT prm DESTINATION lib)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/PRM.h DESTINATION include)
