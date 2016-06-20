cmake_minimum_required(VERSION 2.8)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
get_rospack_flags(rosgraph_msgs)
get_rospack_flags(sttr_msgs)
get_rospack_flags(sttr_srvs)

include_directories(${roscpp_INCLUDE_DIRS}  ${rosgraph_msgs_INCLUDE_DIRS} ${sttr_msgs_INCLUDE_DIRS} ${sttr_srvs_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS} ${rosgraph_msgs_LIBRARY_DIRS} ${sttr_msgs_INCLUDE_DIRS} ${sttr_srvs_INCLUDE_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
add_library(ros_world_plugin SHARED ros_world_plugin.cc)
set_target_properties(ros_world_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}" COMPILE_FLAGS "${rosgraph_msgs_CFLAGS_OTHER}" COMPILE_FLAGS "${sttr_msgs_CFLAGS_OTHER}" COMPILE_FLAGS "${sttr_srvs_CFLAGS_OTHER}")
set_target_properties(ros_world_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}" LINK_FLAGS "${rosgraph_msgs_LDFLAGS_OTHER}" LINK_FLAGS "${sttr_msgs_LDFLAGS_OTHER}" LINK_FLAGS "${sttr_srvs_LDFLAGS_OTHER}")
target_link_libraries(ros_world_plugin ${roscpp_LIBRARIES} ${rosgraph_msgs_LIBRARIES} ${sttr_msgs_LIBRARIES} ${sttr_srvs_LIBRARIES} ${GAZEBO_libraries})
install (TARGETS ros_world_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

