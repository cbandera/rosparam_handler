#include <gtest/gtest.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <rosparam_handler/FooParameters.h>
#include <rosparam_handler/BarParameters.h>

typedef rosparam_handler::FooParameters FooParameters;
typedef rosparam_handler::BarParameters BarParameters;

typedef rosparam_handler::FooConfig ConfigA;
typedef rosparam_handler::BarConfig ConfigB;

class TestRosparamHandlerBase : public testing::Test
{
public:

  TestRosparamHandlerBase() = default;
  ~TestRosparamHandlerBase() = default;

  struct DummyBase
  {
    DummyBase(const std::string& nh_namespace) :
      nh_(nh_namespace) { }

    virtual ~DummyBase() = default;

    template <typename T>
    void init()
    {
      params_ptr_ = boost::make_shared<T>(nh_);
    }

    void fromParamServer()
    {
      params_ptr_->fromParamServer();
    }

    void toParamServer()
    {
      params_ptr_->toParamServer();
    }

    std::string getNamespace()
    {
      return nh_.getNamespace();
    }

    template <typename T>
    void configCallback(T& config, uint32_t /*level*/)
    {
      params_ptr_->fromConfig(config);
    }

    rosparam_handler::ParametersPtr params_ptr_;

    ros::NodeHandle nh_ = ros::NodeHandle("~");
  };

  struct Foo : public DummyBase
  {
    Foo() : DummyBase("foo"), dr_srv_(nh_)
    {
      init<FooParameters>();

      auto cb = boost::bind(&DummyBase::configCallback<ConfigA>, this, _1, _2);
      dr_srv_.setCallback(cb);
    }

    dynamic_reconfigure::Server<ConfigA> dr_srv_;
  };

  struct Bar : public DummyBase
  {
    Bar() : DummyBase("bar"), dr_srv_(nh_)
    {
      init<BarParameters>();

      auto cb = boost::bind(&DummyBase::configCallback<ConfigB>, this, _1, _2);
      dr_srv_.setCallback(cb);
    }

    dynamic_reconfigure::Server<ConfigB> dr_srv_;
  };

  Foo foo_;
  Bar bar_;
};

TEST_F(TestRosparamHandlerBase, Defaults) {

    ASSERT_NO_THROW(foo_.fromParamServer());
    ASSERT_NO_THROW(bar_.fromParamServer());

    boost::shared_ptr<FooParameters> foo_param_ptr;
    boost::shared_ptr<BarParameters> bar_param_ptr;

    // Test casting
    ASSERT_NO_THROW(foo_param_ptr = boost::dynamic_pointer_cast<FooParameters>(foo_.params_ptr_));
    ASSERT_NO_THROW(bar_param_ptr = boost::dynamic_pointer_cast<BarParameters>(bar_.params_ptr_));

    ASSERT_NE(nullptr, foo_param_ptr);
    ASSERT_NE(nullptr, bar_param_ptr);

    // Test casting with helper function
    ASSERT_NO_THROW(foo_param_ptr = rosparam_handler::dynamic_parameters_cast<FooParameters>(foo_.params_ptr_));
    ASSERT_NO_THROW(bar_param_ptr = rosparam_handler::dynamic_parameters_cast<BarParameters>(bar_.params_ptr_));

    ASSERT_NE(nullptr, foo_param_ptr);
    ASSERT_NE(nullptr, bar_param_ptr);

    ASSERT_EQ(1, foo_param_ptr->int_param);
    ASSERT_EQ("param a", foo_param_ptr->str_param);

    ASSERT_EQ("param b", bar_param_ptr->str_param);
    ASSERT_EQ(std::vector<int>({1, 2, 3}), bar_param_ptr->vector_int_param);
}

TEST_F(TestRosparamHandlerBase, DefaultsOnParamServer) {

    ASSERT_NO_THROW(foo_.fromParamServer());
    ASSERT_NO_THROW(bar_.fromParamServer());

    boost::shared_ptr<FooParameters> foo_param_ptr;
    boost::shared_ptr<BarParameters> bar_param_ptr;

    ASSERT_NO_THROW(foo_param_ptr = rosparam_handler::dynamic_parameters_cast<FooParameters>(foo_.params_ptr_));
    ASSERT_NO_THROW(bar_param_ptr = rosparam_handler::dynamic_parameters_cast<BarParameters>(bar_.params_ptr_));

    ASSERT_NE(nullptr, foo_param_ptr);
    ASSERT_NE(nullptr, bar_param_ptr);

    ros::NodeHandle nh("~");

    // values should now be set on param server
    {
        int value;
        ASSERT_TRUE(nh.getParam(foo_.getNamespace() + "/int_param", value));
        ASSERT_EQ(value, foo_param_ptr->int_param);
    }
    {
        std::string value;
        ASSERT_TRUE(nh.getParam(foo_.getNamespace() + "/str_param", value));
        ASSERT_EQ(value, foo_param_ptr->str_param);
    }
    {
        std::vector<int> value;
        ASSERT_TRUE(nh.getParam(bar_.getNamespace() + "/vector_int_param", value));
        ASSERT_EQ(value, bar_param_ptr->vector_int_param);
    }
    {
        std::string value;
        ASSERT_TRUE(nh.getParam(bar_.getNamespace() + "/str_param", value));
        ASSERT_EQ(value, bar_param_ptr->str_param);
    }
}

TEST_F(TestRosparamHandlerBase, SetParamOnServer) {

    ASSERT_NO_THROW(foo_.fromParamServer());
    ASSERT_NO_THROW(bar_.fromParamServer());

    boost::shared_ptr<FooParameters> foo_param_ptr;
    boost::shared_ptr<BarParameters> bar_param_ptr;

    ASSERT_NO_THROW(foo_param_ptr = rosparam_handler::dynamic_parameters_cast<FooParameters>(foo_.params_ptr_));
    ASSERT_NO_THROW(bar_param_ptr = rosparam_handler::dynamic_parameters_cast<BarParameters>(bar_.params_ptr_));

    ASSERT_NE(nullptr, foo_param_ptr);
    ASSERT_NE(nullptr, bar_param_ptr);

    foo_param_ptr->int_param = 9;
    foo_param_ptr->str_param = "Hello";

    bar_param_ptr->vector_int_param = {4,2};
    bar_param_ptr->str_param = "World";

    ASSERT_NO_THROW(foo_.toParamServer());
    ASSERT_NO_THROW(bar_.toParamServer());

    ros::NodeHandle nh("~");

    // values should now be set on param server
    {
        int value;
        ASSERT_TRUE(nh.getParam(foo_.getNamespace() + "/int_param", value));
        ASSERT_EQ(value, foo_param_ptr->int_param);
    }
    {
        std::string value;
        ASSERT_TRUE(nh.getParam(foo_.getNamespace() + "/str_param", value));
        ASSERT_EQ(value, foo_param_ptr->str_param);
    }
    {
        std::vector<int> value;
        ASSERT_TRUE(nh.getParam(bar_.getNamespace() + "/vector_int_param", value));
        ASSERT_EQ(value, bar_param_ptr->vector_int_param);
    }
    {
        std::string value;
        ASSERT_TRUE(nh.getParam(bar_.getNamespace() + "/str_param", value));
        ASSERT_EQ(value, bar_param_ptr->str_param);
    }
}

TEST_F(TestRosparamHandlerBase, DynamicReconf)
{
  ASSERT_NO_THROW(foo_.fromParamServer());
  ASSERT_NO_THROW(bar_.fromParamServer());

  boost::shared_ptr<FooParameters> foo_param_ptr;
  boost::shared_ptr<BarParameters> bar_param_ptr;

  ASSERT_NO_THROW(foo_param_ptr = rosparam_handler::dynamic_parameters_cast<FooParameters>(foo_.params_ptr_));
  ASSERT_NO_THROW(bar_param_ptr = rosparam_handler::dynamic_parameters_cast<BarParameters>(bar_.params_ptr_));

  ASSERT_NE(nullptr, foo_param_ptr);
  ASSERT_NE(nullptr, bar_param_ptr);

  dynamic_reconfigure::ReconfigureRequest  srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::Config conf;

  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = 0;

  dynamic_reconfigure::StrParameter str_param;
  str_param.name = "str_param";
  str_param.value = "dynamic foo string";

  conf.ints.push_back(int_param);
  conf.strs.push_back(str_param);

  srv_req.config = conf;

  ros::service::call(foo_.getNamespace() + "/set_parameters", srv_req, srv_resp);

  conf = dynamic_reconfigure::Config();
  str_param.name = "str_param";
  str_param.value = "dynamic bar string";

  conf.strs.push_back(str_param);

  srv_req.config = conf;

  ros::service::call(bar_.getNamespace() + "/set_parameters", srv_req, srv_resp);

  ros::Duration(1).sleep();

  ASSERT_EQ(0, foo_param_ptr->int_param);
  ASSERT_EQ("dynamic foo string", foo_param_ptr->str_param);

  ASSERT_EQ("dynamic bar string", bar_param_ptr->str_param);
}
