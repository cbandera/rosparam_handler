cmake_minimum_required(VERSION 2.8.3)
project(rosparam_handler)

find_package(catkin REQUIRED)

catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
	# set compiler flags
    find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure rostest roscpp)
    set(ROSPARAM_HANDLER_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR}/cmake)
    include(cmake/rosparam_handler-macros.cmake)
	file(GLOB PROJECT_TEST_FILES_PARAMS RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/cfg/*.params")
	generate_ros_parameter_files(${PROJECT_TEST_FILES_PARAMS})
endif()

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

if (CATKIN_ENABLE_TESTING)
	file(GLOB PROJECT_TEST_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/src/*.cpp")
    set(TEST_TARGET_NAME "rosparam_handler_test")
    add_rostest_gtest(${TEST_TARGET_NAME} test/launch/rosparam_handler.test ${PROJECT_TEST_FILES_SRC})
    add_rostest(test/launch/rosparam_handler_python.test DEPENDENCIES ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(${TEST_TARGET_NAME} ${catkin_LIBRARIES} gtest)
    target_include_directories(${TEST_TARGET_NAME} PUBLIC ${catkin_INCLUDE_DIRS} include)
    add_dependencies(${TEST_TARGET_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    set_property(TARGET ${TEST_TARGET_NAME} PROPERTY CXX_STANDARD 11)
    set_property(TARGET ${TEST_TARGET_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
endif()
