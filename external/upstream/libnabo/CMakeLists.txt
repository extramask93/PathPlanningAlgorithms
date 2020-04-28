find_package(libnabo QUIET)

if(libnabo_FOUND)
  message(STATUS "Found libnabo version")
  add_library(libnabo_LIBRARY INTERFACE)
endif()
