cmake_minimum_required(VERSION 3.0.2)
project(planner_interface_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Binaries
cs_add_executable(planner_interface_node src/planner_interface.cc)
# target_link_libraries(planner_interface_node ${PROJECT_NAME})

# Export
cs_install()
cs_export()
