# How to Write Your First .params File
**Description**: This tutorial will familiarize you with .params files that allow you to use rosparam_handler.
**Tutorial Level**: INTERMEDIATE

## Basic Setup

To begin lets create a package called rosparam_tutorials which depends on rospy, roscpp, rosparam_handler and dynamic_reconfigure.
```shell
catkin_create_pkg --rosdistro ROSDISTRO rospy roscpp rosparam_handler dynamic_reconfigure
```
Where ROSDISTRO is the ROS distribution you are using.

Now in your package create a cfg directory, this is where all cfg and params files live:
```shell
mkdir cfg
```
Lastly you will need to create a params file, for this example we'll call it Tutorials.params, and open it in your favorite editor.

## The params File

Add the following to your Tutorials.params file:
```python
#!/usr/bin/env python
from rosparam_handler.parameter_generator_catkin import *
gen = ParameterGenerator()

# Parameters with different types
gen.add("int_param", paramtype="int", description="An Integer parameter")
gen.add("double_param", paramtype="double",description="A double parameter")
gen.add("str_param", paramtype="std::string", description="A string parameter",  default="Hello World")
gen.add("bool_param", paramtype="bool", description="A Boolean parameter")
gen.add("vector_param", paramtype="std::vector<double>", description="A vector parameter")
gen.add("map_param", paramtype="std::map<std::string,std::string>", description="A map parameter")

# Default min and max values
gen.add("weight", paramtype="double",description="Weight can not be negative", min=0.0)
gen.add("age", paramtype="int",description="Normal age of a human is inbetween 0 and 100", min=0, max=100)
gen.add("default_param", paramtype="std::string",description="Parameter with default value", default="Hello World")
# Default vector/map
gen.add("vector_bool", paramtype="std::vector<bool>", description="A vector of boolean with default value.", default=[False, True, True, False, True])
gen.add("map_string_float", paramtype="std::map<std::string,float>", description="A map of <std::string,float> with default value.", default={"a":0.1, "b":1.2, "c":2.3, "d":3.4, "e":4.5})

# Constant and configurable parameters
gen.add("optimal_parameter", paramtype="double", description="Optimal parameter, can not be set via rosparam", default=10, constant=True)
gen.add("configurable_parameter", paramtype="double", description="This parameter can be set via dynamic_reconfigure", configurable=True)

# Defining the namespace
gen.add("global_parameter", paramtype="std::string", description="This parameter is defined in the global namespace", global_scope=True)

# Full signature
gen.add("dummy", paramtype="double", description="My Dummy parameter", level=0,
edit_method="", default=5.2, min=0, max=10, configurable=True,
global_scope=False, constant=False)

# Add an enum:
gen.add_enum("my_enum", description="My first self written enum",
entry_strings=["Small", "Medium", "Large", "ExtraLarge"], default="Medium"))

# Add a subgroup
my_group = gen.add_group("my_group")
my_group.add("subparam", paramtype="std::string", description="This parameter is part of a group", configurable=True)

#Syntax : Package, Node, Config Name(The final name will be MyDummyConfig)
exit(gen.generate("rosparam_tutorials", "example_node", "Tutorial"))
```

Now lets break the code down line by line.
```python
#!/usr/bin/env python
from rosparam_handler.parameter_generator_catkin import *
gen = ParameterGenerator()
```
This first lines are pretty simple, they just initialize ros, import and instantiate the parameter generator.

Now that we have a generator we can start to define parameters. The add function adds a parameter to the list of parameters. It takes a the following mandatory arguments:

- **name**: a string which specifies the name under which this parameter should be stored
- **paramtype**: defines the type of value stored, and can be any of the primitive types: "int", "double", "std::string", "bool" or a container type using one of the primitive types: "std::vector<...>", "std::map<std::string, ...>"
- **description**: string which describes the parameter

Furthermore, following optional arguments can be passed:
- **configurable**: Make this parameter reconfigurable at run time. Default: False
- **global_scope**: Make this parameter live in the global namespace. Default: False
- **level**: A bitmask which will later be passed to the dynamic reconfigure callback. When the callback is called all of the level values for parameters that have been changed are ORed together and the resulting value is passed to the callback. This is only used when *configurable* is set to True.
- **edit_method**: An optional string that is directly passed to dynamic reconfigure. This is only used when *configurable* is set to True.
- **default**: specifies the default value. Can not be set for global parameters.
- **min**: specifies the min value (optional and does not apply to strings and bools)
- **max**: specifies the max value (optional and does not apply to strings and bools)

```python
# Parameters with different types
gen.add("int_param", paramtype="int", description="An Integer parameter")
gen.add("double_param", paramtype="double",description="A double parameter")
gen.add("str_param", paramtype="std::string", description="A string parameter",  "Hello World")
gen.add("bool_param", paramtype="bool", description="A Boolean parameter")
gen.add("vector_param", paramtype="std::vector<double>", description="A vector parameter")
gen.add("map_param", paramtype="std::map<std::string,std::string>", description="A map parameter")
```

These lines simply define parameters of the different types. Their values will be retrieved from the ros parameter server.

```python
# Default min and max values
gen.add("weight", paramtype="double",description="Weight can not be negative", min=0.0)
gen.add("age", paramtype="int",description="Normal age of a human is inbetween 0 and 100", min=0, max=100)
gen.add("default_param", paramtype="std::string",description="Parameter with default value", default="Hello World")
```

These lines add parameters, which either have default values, which means that they wont throw an error if the parameters are not set on the server, or they have constraints on min/max values for the retrieved parameters. These bounds will be enforced, when fetching parameters from the server.

```python
# Constant and configurable parameters
gen.add("optimal_parameter", paramtype="double", description="Optimal parameter, can not be set via rosparam", default=10, constant=True)
gen.add("configurable_parameter", paramtype="double", description="This parameter can be set via dynamic_reconfigure", configurable_parameter=True)
```

These lines define a parameter that is configurable and one that is constant. Configurable means, that the entry will be added to the dynamic_reconfigure config file and can later be changed at runtime. Vectors, maps and global parameters can not be set configurable.
A constant parameter can not be set through the parameter server anymore. This can be useful, if you have found optimal parameters for your setup, which should not be changed by users anymore.

```python
# Defining the namespace
gen.add("global_parameter", paramtype="std::string", description="This parameter is defined in the global namespace", global_scope=True)
```

With the global_scope flag, parameters can be defined to live in the global namespace. Normally you should always keep your parameters in the private namespace of a node. Only exception is when several nodes need to get the exact same value for a parameter. (E.g. a common map file)

```python
# Add an enum:
gen.add_enum("my_enum", description="My first self written enum",
entry_strings=["Small", "Medium", "Large", "ExtraLarge"], default="Medium"))
```

By using the add_enum function, an enum for the dynamic_reconfigure window can be easily defined. The entry_strings will also be static parameters of the resulting parameter struct.

```python
# Add a subgroup
my_group = gen.add_group("my_group")
my_group.add("subparam", paramtype="std::string", description="This parameter is part of a group", configurable=True)
```

Finally, by using the add_group function, you can sort parameters into groups in the dynamic_reconfigure window. This obviously only makes sense for configurable parameters.

```python
exit(gen.generate("rosparam_tutorials", "example_node", "Tutorial"))
```

The last line simply tells the generator to generate the necessary files and exit the program. The second parameter is the name of a node this could run in (used to generate documentation only), the third parameter is a name prefix the generated files will get (e.g. "<name>Config.h" for the dynamic_reconfigure struct and "<name>Parameters.h" for the parameter struct.

NOTE: The third parameter should be equal to the params file name, without extension. Otherwise the libraries will be generated in every build, forcing a recompilation of the nodes which use them.

## Add params file to CMakeLists

In order to make this params file usable it must be executable, so lets use the following command to make it excecutable

```shell
chmod a+x cfg/Tutorials.params
```

Next we need to add the following lines to our CMakeLists.txt.

```cmake
#add rosparam_handler api
find_package(catkin REQUIRED rosparam_handler dynamic_reconfigure)

generate_ros_parameter_files(
  cfg/Tutorials.params
  cfg/SomeOtherCfgFile.cfg
  #...
)

# make sure configure headers are built before any node using them
add_dependencies(example_node ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
add_dependencies(example_node ${PROJECT_NAME}_genparam) # For rosparam_handler
```
Note: You need a node example_node that is already built, before the add_dependencies line is reached (ref Create a node in C++).  

This will run our params file when the package is built. The last thing to do is to build the package and we're done!

Note: It should be noted here, that you have to pass **all** '.params' **and all** '.cfg' files to the generate_parameter_files call. This is because dynamic_reconfigure can only be called once per package. Your normal cfg files will be passed on together with the newly created cfg files.

For information on how to use the resulting parameter struct in your code, see the next tutorial on [How to use your parameter struct](HowToUseYourParameterStruct.md).
