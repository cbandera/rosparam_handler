#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosparam_handler/MinMaxParameters.h>

typedef rosparam_handler::MinMaxParameters ParamType;
typedef rosparam_handler::MinMaxConfig ConfigType;

TEST(RosparamHandler, MinMax) {
    ParamType testParams(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testParams.fromParamServer());

    ASSERT_EQ(2, testParams.int_param_w_minmax);
    ASSERT_DOUBLE_EQ(2., testParams.double_param_w_minmax);

    ASSERT_EQ(std::vector<int>({0, 2, 2}), testParams.vector_int_param_w_minmax);
    ASSERT_EQ(std::vector<double>({0., 1.2, 2.}), testParams.vector_double_param_w_minmax);

    std::map<std::string, double> tmp{{"value1", 0.}, {"value2", 1.2}, {"value3", 2.}};
    ASSERT_EQ(tmp, testParams.map_param_w_minmax);
}