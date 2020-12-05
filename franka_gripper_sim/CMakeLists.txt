cmake_minimum_required(VERSION 3.10)

project(franka_gripper_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  franka_gripper
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 actionlib
)

add_executable(franka_gripper_sim_node
  src/franka_gripper_sim_node.cpp
  src/franka_gripper_sim.cpp
)

add_dependencies(franka_gripper_sim_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_gripper_sim_node PRIVATE
  ${catkin_LIBRARIES}
)

target_include_directories(franka_gripper_sim_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)

install(TARGETS ${${CMAKE_PROJECT_NAME}_node}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
