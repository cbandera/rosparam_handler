# rosparam_handler
## Package Summary
A unified parameter handler for nodes with automatic code generation.
Save time on defining your parameters. No more redundant code. Easy error checking. Make them dynamic with a single flag.

- Maintainer status: maintained
- Maintainer: Claudio Bandera <cbandera@posteo.de>
- Author: Claudio Bandera <cbandera@posteo.de>
- License: BSD
- Bug / feature tracker: https://github.com/cbandera/rosparam_handler/issues
- Source: git https://github.com/cbandera/rosparam_handler.git (branch: master)


## Unified Parameter Handling for ROS
When working with ROS and Parameters, there are a couple of tools to help you with handing Parameters to your nodes and to modify them, e.g. [Parameter Server](http://wiki.ros.org/Parameter%20Server) and [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure/).

But with the multitude of options on where to specify your parameters, we often face the problem that we get a redundancy in our code and config files.

The `rosparam_handler` let's you:
- specify all of your parameters in a single file
- use a generated struct to hold your parameters
- use member method for grabbing the parameters from the parameter server
- use member method for updating them from dynamic_reconfigure.
- make your parameters configurable with a single flag.
- set default, min and max values
- choose between global and private namespace
- save a lot of time on specifying your parameters in several places.

## Usage
See the Tutorials on
- [How to write your first .params file](doc/HowToWriteYourFirstParamsFile.md)
- [How to use your parameter struct](doc/HowToUseYourParameterStruct.md)

## Credits
This project uses Open Source components. The code was, in large parts, built upon such existing open source software. You can find the source code of their projects along with license information below. We acknowledge and are grateful to these developers for their contributions to open source.

Project: [dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure)  
Copyright 2015, Open Source Robotics Foundation, Inc.  
License: [3-Clause BSD](https://github.com/ros/dynamic_reconfigure/blob/master/src/dynamic_reconfigure/parameter_generator_catkin.py)
