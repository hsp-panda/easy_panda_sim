cmake_minimum_required(VERSION 3.17)

project(easy_panda_sim)

add_compile_options(-std=c++14)
set(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GAZEBO_CXX_FLAGS}" )

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    effort_controllers
    gazebo_ros
    joint_state_controller
    joint_state_publisher
    joint_trajectory_controller
    robot_state_publisher
    roscpp
    std_msgs
    tf
    xacro
)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
message(${JSONCPP_LIBRARIES})

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS xacro)

catkin_package(CATKIN_DEPENDS xacro)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
