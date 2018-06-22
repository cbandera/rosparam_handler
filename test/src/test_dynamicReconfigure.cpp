#include <gtest/gtest.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <rosparam_handler/DefaultsParameters.h>

typedef rosparam_handler::DefaultsParameters ParamType;
typedef rosparam_handler::DefaultsConfig ConfigType;

class TestDynamicReconfigure : public testing::Test
{
public:

  TestDynamicReconfigure()
    : nh("~")
    , testParams(nh)
    , drSrv(nh)
  {
    auto cb = boost::bind(&TestDynamicReconfigure::configCallback, this, _1, _2);
    drSrv.setCallback(cb);
  }

  ~TestDynamicReconfigure() = default;

  void configCallback(ConfigType& config, uint32_t /*level*/)
  {
    testParams.fromConfig(config);
  }

  void rosparamToConfig(ConfigType& config)
  {
    testParams.toConfig(config);
  }

  ros::NodeHandle nh;

  ParamType testParams;

  dynamic_reconfigure::Server<ConfigType> drSrv;
};

TEST_F(TestDynamicReconfigure, DynamicReconf)
{
  // Delete in case they are still on the server.
  nh.deleteParam("int_param_w_default");
  nh.deleteParam("double_param_w_default");
  nh.deleteParam("str_param_w_default");
  nh.deleteParam("bool_param_w_default");

  ASSERT_NO_THROW(testParams.fromParamServer());

  ASSERT_EQ(1, testParams.int_param_w_default);
  ASSERT_DOUBLE_EQ(1.1, testParams.double_param_w_default);
  ASSERT_EQ("Hello World", testParams.str_param_w_default);
  ASSERT_TRUE(testParams.bool_param_w_default);

  // ASSERT_EQ(std::vector<int>({1, 2, 3}), testParams.vector_int_param_w_default);
  // ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testParams.vector_double_param_w_default);
  // ASSERT_EQ(std::vector<bool>({false, true}), testParams.vector_bool_param_w_default);
  // ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testParams.vector_string_param_w_default);

  // std::map<std::string, std::string> tmp{{"Hello", "World"}};
  // ASSERT_EQ(tmp, testParams.map_param_w_default);

  // ASSERT_EQ(1, testParams.enum_param_w_default);

  dynamic_reconfigure::ReconfigureRequest  srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::Config conf;

  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param_w_default";
  int_param.value = 2;

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param_w_default";
  double_param.value = 2.2;

  dynamic_reconfigure::StrParameter str_param;
  str_param.name = "str_param_w_default";
  str_param.value = "Foo Bar";

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "bool_param_w_default";
  bool_param.value = false;

  conf.ints.push_back(int_param);
  conf.doubles.push_back(double_param);
  conf.strs.push_back(str_param);
  conf.bools.push_back(bool_param);

  srv_req.config = conf;

  ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv_req, srv_resp));

  ros::Duration(1).sleep();

  EXPECT_EQ(2, testParams.int_param_w_default);
  EXPECT_DOUBLE_EQ(2.2, testParams.double_param_w_default);
  EXPECT_EQ("Foo Bar", testParams.str_param_w_default);
  EXPECT_FALSE(testParams.bool_param_w_default);
}

TEST_F(TestDynamicReconfigure, ToDynamicReconf)
{
  ConfigType config;

  rosparamToConfig(config);

  EXPECT_EQ(testParams.int_param_w_default, config.int_param_w_default);
  EXPECT_DOUBLE_EQ(testParams.double_param_w_default, config.double_param_w_default);
  EXPECT_EQ(testParams.str_param_w_default, config.str_param_w_default);
  EXPECT_FALSE(config.bool_param_w_default);
}
