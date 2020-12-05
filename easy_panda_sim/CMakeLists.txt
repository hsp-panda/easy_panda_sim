cmake_minimum_required(VERSION 3.10)

project(easy_panda_sim)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
