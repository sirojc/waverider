cmake_minimum_required(VERSION 3.0.2)
project(waverider_chomp_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES
    geometry_msgs
    message_generation
    std_msgs
    nav_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
${CATKIN_PACKAGE_DEPENDENCIES}
)

add_service_files(
    FILES
        GetTraj.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
${CATKIN_PACKAGE_DEPENDENCIES}
)
