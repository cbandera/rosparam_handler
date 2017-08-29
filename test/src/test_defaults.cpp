#include <ros/ros.h>
#include <gtest/gtest.h>
#include <rosparam_handler/DefaultsParameters.h>

typedef rosparam_handler::DefaultsParameters ParamType;
typedef rosparam_handler::DefaultsConfig ConfigType;

TEST(RosparamHandler, Defaults) {
    ParamType testParams(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testParams.fromParamServer());

    ASSERT_EQ(1, testParams.int_param_w_default);
    ASSERT_DOUBLE_EQ(1.1, testParams.double_param_w_default);
    ASSERT_EQ("Hello World", testParams.str_param_w_default);
    ASSERT_EQ(true, testParams.bool_param_w_default);

    ASSERT_EQ(std::vector<int>({1,2,3}), testParams.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testParams.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testParams.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testParams.vector_string_param_w_default);

    std::map<std::string,std::string> tmp{{"Hello","World"}};
    ASSERT_EQ(tmp, testParams.map_param_w_default);

    ASSERT_EQ(1, testParams.enum_param_w_default);
}

TEST(RosparamHandler, DefaultsOnParamServer) {
    ros::NodeHandle nh("~");
    ParamType testParams(nh);
    ASSERT_NO_THROW(testParams.fromParamServer());

    // values should now be set on param server
    {
        int int_param;
        ASSERT_TRUE(nh.getParam("int_param_w_default", int_param));
        ASSERT_EQ(int_param, testParams.int_param_w_default);
    }
    {
        double double_param;
        ASSERT_TRUE(nh.getParam("double_param_w_default", double_param));
        EXPECT_EQ(double_param, testParams.double_param_w_default);
    }
    {
        bool bool_param;
        ASSERT_TRUE(nh.getParam("bool_param_w_default", bool_param));
        EXPECT_EQ(bool_param, testParams.bool_param_w_default);
    }
    {
        std::vector<int> vector_int_param;
        ASSERT_TRUE(nh.getParam("vector_int_param_w_default", vector_int_param));
        EXPECT_EQ(vector_int_param, testParams.vector_int_param_w_default);
    }
    {
        std::vector<double> vector_double_param;
        ASSERT_TRUE(nh.getParam("vector_double_param_w_default", vector_double_param));
        EXPECT_EQ(vector_double_param, testParams.vector_double_param_w_default);
    }
    {
        std::vector<bool> vector_bool_param;
        ASSERT_TRUE(nh.getParam("vector_bool_param_w_default", vector_bool_param));
        EXPECT_EQ(vector_bool_param, testParams.vector_bool_param_w_default);
    }
    {
        std::vector<std::string> vector_string_param;
        ASSERT_TRUE(nh.getParam("vector_string_param_w_default", vector_string_param));
        EXPECT_EQ(vector_string_param, testParams.vector_string_param_w_default);
    }
    {
        std::map<std::string, std::string> map_param_w_default;
        ASSERT_TRUE(nh.getParam("map_param_w_default", map_param_w_default));
        EXPECT_EQ(map_param_w_default, testParams.map_param_w_default);
    }
    {
        int enum_param;
        ASSERT_TRUE(nh.getParam("enum_param_w_default", enum_param));
        EXPECT_EQ(enum_param, testParams.enum_param_w_default);
    }
}
