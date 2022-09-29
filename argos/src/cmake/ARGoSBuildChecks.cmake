#
# Check ARGoS
#
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})


#
# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
#
#include(ARGoSCheckQTOpenGL)

#
# Find Lua
#
#find_package(Lua53 REQUIRED)

#
# Look for librt, necessary on some platforms
#
find_package(RT)

#
# Set ARGoS include dir
#
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

#
# Set ARGoS link dir
#
link_directories(${ARGOS_LIBRARY_DIRS})
