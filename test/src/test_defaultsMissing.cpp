#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosparam_handler/DefaultsMissingParameters.h>

typedef rosparam_handler::DefaultsMissingParameters ParamType;
typedef rosparam_handler::DefaultsMissingConfig ConfigType;

TEST(RosparamHandler, DefaultsMissing) {
    ParamType testParams(ros::NodeHandle("~"));
    ASSERT_THROW(testParams.fromParamServer(), std::runtime_error);
}
