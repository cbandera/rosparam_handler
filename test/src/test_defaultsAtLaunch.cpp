#include <ros/ros.h>
#include <gtest/gtest.h>
#include <rosparam_handler/DefaultsAtLaunchParameters.h>

typedef rosparam_handler::DefaultsAtLaunchParameters ParamType;
typedef rosparam_handler::DefaultsAtLaunchConfig ConfigType;

TEST(RosparamHandler, DefaultsAtLaunch) {
    ParamType testParams(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testParams.fromParamServer());

    ASSERT_EQ(1, testParams.int_param_wo_default);
    ASSERT_DOUBLE_EQ(1.1, testParams.double_param_wo_default);
    ASSERT_EQ("Hello World", testParams.str_param_wo_default);
    ASSERT_EQ(true, testParams.bool_param_wo_default);

    ASSERT_EQ(std::vector<int>({1,2,3}), testParams.vector_int_param_wo_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testParams.vector_double_param_wo_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testParams.vector_bool_param_wo_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testParams.vector_string_param_wo_default);

	std::map<std::string,std::string> tmp{{"Hello","World"}};
    ASSERT_EQ(tmp, testParams.map_param_wo_default);

    ASSERT_EQ(1, testParams.enum_param_wo_default);
}