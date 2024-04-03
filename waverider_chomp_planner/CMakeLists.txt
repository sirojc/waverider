cmake_minimum_required(VERSION 3.0.2)
project(waverider_chomp_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Binaries
cs_add_executable(waverider_chomp_planner_node src/waverider_chomp_planner.cc)
# target_link_libraries(waverider_chomp_planner_node ${PROJECT_NAME})

# Export
cs_install()
cs_export()

