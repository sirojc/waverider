cmake_minimum_required(VERSION 3.0.2)
project(waverider_chomp_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES
    geometry_msgs
    message_generation
    std_msgs
    nav_msgs
    navigation_msgs
    actionlib_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
${CATKIN_PACKAGE_DEPENDENCIES}
)

add_action_files(
  DIRECTORY
    action
  FILES
    PlanToGoalLocalGuidance.action
    PlanToGoalWaverider.action
)

add_service_files(
    FILES
        GetTraj.srv
        SetPlannerType.srv
)

add_message_files(
  FILES
    PlannerType.msg
    RobotState.msg
    Vector2.msg
    EstimatedState.msg
    DesiredLinearAccelerationCalculation.msg
    DesiredYawVelocityCalculation.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
    navigation_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
${CATKIN_PACKAGE_DEPENDENCIES}
)
