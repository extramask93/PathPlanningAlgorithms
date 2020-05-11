add_library(ga "")
target_sources(ga
        PRIVATE
        GeneticAlgorithm.cpp
        Utils.cpp
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/GeneticAlgorithm.cpp
        )
target_include_directories(ga
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(ga utils)
