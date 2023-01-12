include(CMakeFindDependencyMacro)

if(NOT TARGET csrtsi)
  include("${CMAKE_CURRENT_LIST_DIR}/csrtsiTargets.cmake")
endif()

# This is for catkin compatibility. Better use target_link_libraries(<my_target> cs_rtsi)
set(cs_rtsi_LIBRARIES csrtsi)
get_target_property(cs_rtsi_INCLUDE_DIRS csrtsi INTERFACE_INCLUDE_DIRECTORIES)