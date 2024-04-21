cmake_minimum_required(VERSION 3.0.2)
project(waverider_chomp_planner)

find_package(
catkin
REQUIRED
  navigation_common
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    navigation_common
    tf2
    tf2_geometry_msgs
    tf2_ros
#  DEPENDS system_lib
)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Binaries
cs_add_executable(waverider_chomp_planner_node src/waverider_chomp_planner.cc)
# target_link_libraries(waverider_chomp_planner_node ${PROJECT_NAME})

cs_add_executable(waverider_planner_node src/waverider_planner.cc)
target_link_libraries(waverider_planner_node ${catkin_LIBRARIES})

# Export
cs_install()
cs_export()

