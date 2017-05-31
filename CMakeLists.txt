cmake_minimum_required(VERSION 2.8.3)
project(rosparam_handler)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS catkin
        CFG_EXTRAS rosparam_handler-extras.cmake
)

install(
        DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        DIRECTORY templates
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        DIRECTORY include/rosparam_handler
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)