cmake_minimum_required(VERSION 3.10)

project(easy_panda_sim)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    joint_state_controller
    joint_state_publisher
    joint_trajectory_controller
    robot_state_publisher
    xacro
)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
