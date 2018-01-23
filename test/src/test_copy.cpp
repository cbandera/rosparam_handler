#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosparam_handler/DefaultsParameters.h>

typedef rosparam_handler::DefaultsParameters ParamType;

TEST(RosparamHandler, CopyInit) {
    ParamType testParams(ros::NodeHandle("~copy_init"));
    ASSERT_NO_THROW(testParams.fromParamServer());

    ASSERT_EQ(1, testParams.int_param_w_default);
    ASSERT_DOUBLE_EQ(1.1, testParams.double_param_w_default);
    ASSERT_EQ("Hello World", testParams.str_param_w_default);
    ASSERT_TRUE(testParams.bool_param_w_default);

    ASSERT_EQ(std::vector<int>({1, 2, 3}), testParams.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testParams.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testParams.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testParams.vector_string_param_w_default);

    std::map<std::string, std::string> tmp{{"Hello", "World"}};
    ASSERT_EQ(tmp, testParams.map_param_w_default);

    ASSERT_EQ(1, testParams.enum_param_w_default);

    testParams.int_param_w_default           = 2;
    testParams.double_param_w_default        = 2.2;
    testParams.str_param_w_default           = "Foo Bar";
    testParams.bool_param_w_default          = false;
    testParams.vector_int_param_w_default    = {4, 5, 6};
    testParams.vector_double_param_w_default = {1.4, 1.5, 1.6};
    testParams.vector_bool_param_w_default   = {true, false};
    testParams.vector_string_param_w_default = {"Foo", "Bar"};
    testParams.map_param_w_default           = {{"Foo", "Bar"}};
    testParams.enum_param_w_default          = 2;

    ParamType testParamsCopy(testParams);

    ASSERT_EQ(2, testParamsCopy.int_param_w_default);
    ASSERT_DOUBLE_EQ(2.2, testParamsCopy.double_param_w_default);
    ASSERT_EQ("Foo Bar", testParamsCopy.str_param_w_default);
    ASSERT_FALSE(testParamsCopy.bool_param_w_default);

    ASSERT_EQ(std::vector<int>({4, 5, 6}), testParamsCopy.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.4, 1.5, 1.6}), testParamsCopy.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({true, false}), testParamsCopy.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Foo", "Bar"}), testParamsCopy.vector_string_param_w_default);

    tmp = {{"Foo", "Bar"}};
    ASSERT_EQ(tmp, testParamsCopy.map_param_w_default);

    ASSERT_EQ(2, testParamsCopy.enum_param_w_default);
}

TEST(RosparamHandler, CopyAssign) {
    ParamType testParams(ros::NodeHandle("~copy_assign"));
    ASSERT_NO_THROW(testParams.fromParamServer());

    ASSERT_EQ(1, testParams.int_param_w_default);
    ASSERT_DOUBLE_EQ(1.1, testParams.double_param_w_default);
    ASSERT_EQ("Hello World", testParams.str_param_w_default);
    ASSERT_TRUE(testParams.bool_param_w_default);

    ASSERT_EQ(std::vector<int>({1, 2, 3}), testParams.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testParams.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testParams.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testParams.vector_string_param_w_default);

    std::map<std::string, std::string> tmp{{"Hello", "World"}};
    ASSERT_EQ(tmp, testParams.map_param_w_default);

    ASSERT_EQ(1, testParams.enum_param_w_default);

    ParamType testParamsCopy(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testParamsCopy.fromParamServer());

    ASSERT_EQ(1, testParamsCopy.int_param_w_default);
    ASSERT_DOUBLE_EQ(1.1, testParamsCopy.double_param_w_default);
    ASSERT_EQ("Hello World", testParamsCopy.str_param_w_default);
    ASSERT_TRUE(testParamsCopy.bool_param_w_default);

    ASSERT_EQ(std::vector<int>({1, 2, 3}), testParamsCopy.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testParamsCopy.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testParamsCopy.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testParamsCopy.vector_string_param_w_default);

    ASSERT_EQ(tmp, testParamsCopy.map_param_w_default);

    ASSERT_EQ(1, testParamsCopy.enum_param_w_default);

    testParams.int_param_w_default           = 2;
    testParams.double_param_w_default        = 2.2;
    testParams.str_param_w_default           = "Foo Bar";
    testParams.bool_param_w_default          = false;
    testParams.vector_int_param_w_default    = {4, 5, 6};
    testParams.vector_double_param_w_default = {1.4, 1.5, 1.6};
    testParams.vector_bool_param_w_default   = {true, false};
    testParams.vector_string_param_w_default = {"Foo", "Bar"};
    testParams.map_param_w_default           = {{"Foo", "Bar"}};
    testParams.enum_param_w_default          = 2;

    testParamsCopy = testParams;

    ASSERT_EQ(2, testParamsCopy.int_param_w_default);
    ASSERT_DOUBLE_EQ(2.2, testParamsCopy.double_param_w_default);
    ASSERT_EQ("Foo Bar", testParamsCopy.str_param_w_default);
    ASSERT_FALSE(testParamsCopy.bool_param_w_default);

    ASSERT_EQ(std::vector<int>({4, 5, 6}), testParamsCopy.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.4, 1.5, 1.6}), testParamsCopy.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({true, false}), testParamsCopy.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Foo", "Bar"}), testParamsCopy.vector_string_param_w_default);

    tmp = {{"Foo", "Bar"}};
    ASSERT_EQ(tmp, testParamsCopy.map_param_w_default);

    ASSERT_EQ(2, testParamsCopy.enum_param_w_default);
}
