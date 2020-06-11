add_library(ants "")

add_library(planning::ants ALIAS ants)
target_sources(ants
        PRIVATE
        AntColony.cpp
        Utils.cpp
        )
#------------------------------------
find_package (Boost 1.36 REQUIRED)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message("Boost libraries not found")
endif()
#-----------------------------------
target_include_directories(ants
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>

        )
target_link_libraries(ants utils iplanner)

install(TARGETS ants EXPORT planning-export DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CMAKE_CURRENT_LIST_DIR}/AntColony.h ${CMAKE_CURRENT_LIST_DIR}/Utils.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
