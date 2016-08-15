cmake_minimum_required(VERSION 2.8.3)
project(rosparam_handler)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS catkin
        CFG_EXTRAS rosparam_handler-extras.cmake
)

install(
        DIRECTORY cmake/Modules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)

install(
        DIRECTORY templates
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
