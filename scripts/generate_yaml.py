#!/usr/bin/env python
import sys
import os
from rosparam_handler.parameter_generator_catkin import YamlGenerator


def generate_yaml(in_file):

    if not os.path.exists(in_file):
        sys.exit('ERROR: Param file %s was not found!' % in_file)
    # Read argument
    in_file = os.path.abspath(in_file)
    basename = os.path.basename(in_file)
    basename, _ = os.path.splitext(basename)
    out_file = os.path.join(os.getcwd(), basename + "Parameters.yaml")
    print("Yaml file is written to: {}".format(out_file))

    # Mock call arguments for call to ParamGenerator
    sys.argv[1:4] = ["", "", "", ""]
    # Redirect import statement
    sys.modules['rosparam_handler.parameter_generator_catkin'] = __import__('sys')

    # Execute param file in mocked environment
    global_vars = {"__file__": in_file, 'ParameterGenerator': YamlGenerator}
    return execfile(in_file, global_vars)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit('Usage: %s <.params file>' % sys.argv[0])

    generate_yaml(sys.argv[1])
