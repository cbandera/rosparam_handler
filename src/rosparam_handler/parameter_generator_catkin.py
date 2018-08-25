# Copyright (c) 2016, Claudio Bandera
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the organization nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Claudio Bandera
#

from __future__ import print_function
from string import Template
import sys
import os
import re


def eprint(*args, **kwargs):
    print("************************************************", file=sys.stderr, **kwargs)
    print("Error when setting up parameter '{}':".format(args[0]), file=sys.stderr, **kwargs)
    print(*args[1:], file=sys.stderr, **kwargs)
    print("************************************************", file=sys.stderr, **kwargs)
    sys.exit(1)


class ParameterGenerator(object):
    """Automatic config file and header generator"""

    def __init__(self, parent=None, group=""):
        """Constructor for ParamGenerator"""
        self.enums = []
        self.parameters = []
        self.childs = []
        self.parent = parent
        if group:
            self.group = group
        else:
            self.group = "gen"
        self.group_variable = filter(str.isalnum, self.group)

        if len(sys.argv) != 5:
            eprint(
                "ParameterGenerator: Unexpected amount of args, did you try to call this directly? You shouldn't do this!")

        self.dynconfpath = sys.argv[1]
        self.share_dir = sys.argv[2]
        self.cpp_gen_dir = sys.argv[3]
        self.py_gen_dir = sys.argv[4]

        self.pkgname = None
        self.nodename = None
        self.classname = None

    def add_group(self, name):
        """
        Add a new group in the dynamic reconfigure selection
        :param name: name of the new group
        :return: a new group object that you can add parameters to
        """
        if not name:
            eprint("You have added a group with an empty group name. This is not supported!")
        child = ParameterGenerator(self, name)
        self.childs.append(child)
        return child

    def add_enum(self, name, description, entry_strings, default=None):
        """
        Add an enum to dynamic reconfigure
        :param name: Name of enum parameter
        :param description: Informative documentation string
        :param entry_strings: Enum entries, must be strings! (will be numbered with increasing value)
        :param default: Default value
        :return:
        """

        entry_strings = [str(e) for e in entry_strings]  # Make sure we only get strings
        if default is None:
            default = 0
        else:
            default = entry_strings.index(default)
        self.add(name=name, paramtype="int", description=description, edit_method=name, default=default,
                 configurable=True)
        for e in entry_strings:
            self.add(name=name + "_" + e, paramtype="int", description="Constant for enum {}".format(name),
                     default=entry_strings.index(e), constant=True)
        self.enums.append({'name': name, 'description': description, 'values': entry_strings})

    def add(self, name, paramtype, description, level=0, edit_method='""', default=None, min=None, max=None,
            configurable=False, global_scope=False, constant=False):
        """
        Add parameters to your parameter struct. Call this method from your .params file!

        - If no default value is given, you need to specify one in your launch file
        - Global parameters, vectors, maps and constant params can not be configurable

        :param self:
        :param name: The Name of you new parameter
        :param paramtype: The C++ type of this parameter. Can be any of ['std::string', 'int', 'bool', 'float',
        'double'] or std::vector<...> or std::map<std::string, ...>
        :param description: Choose an informative documentation string for this parameter.
        :param level: (optional) Passed to dynamic_reconfigure
        :param edit_method: (optional) Passed to dynamic_reconfigure
        :param default: (optional) default value
        :param min: (optional)
        :param max: (optional)
        :param configurable: (optional) Should this parameter be dynamic configurable
        :param global_scope: (optional) If true, parameter is searched in global ('/') namespace instead of private (
        '~') ns
        :param constant: (optional) If this is true, the parameter will not be fetched from param server,
        but the default value is kept.
        :return: None
        """
        configurable = self._make_bool(configurable)
        global_scope = self._make_bool(global_scope)
        constant = self._make_bool(constant)
        newparam = {
            'name': name,
            'type': paramtype,
            'default': default,
            'level': level,
            'edit_method': edit_method,
            'description': description,
            'min': min,
            'max': max,
            'is_vector': False,
            'is_map': False,
            'configurable': configurable,
            'constant': constant,
            'global_scope': global_scope,
        }
        self._perform_checks(newparam)
        self.parameters.append(newparam)

    def _perform_checks(self, param):
        """
        Will test some logical constraints as well as correct types.
        Throws Exception in case of error.
        :param self:
        :param param: Dictionary of one param
        :return:
        """

        in_type = param['type'].strip()
        if param['max'] is not None or param['min'] is not None:
            if in_type in ["std::string", "bool"]:
                eprint(param['name'], "Max and min can not be specified for variable of type %s" % in_type)

        if in_type.startswith('std::vector'):
            param['is_vector'] = True
        if in_type.startswith('std::map'):
            param['is_map'] = True

        if (param['is_vector']):
            if (param['max'] is not None or param['min'] is not None):
                ptype = in_type[12:-1].strip()
                if ptype == "std::string":
                    eprint(param['name'], "Max and min can not be specified for variable of type %s" % in_type)

        if (param['is_map']):
            if (param['max'] is not None or param['min'] is not None):
                ptype = in_type[9:-1].split(',')
                if len(ptype) != 2:
                    eprint(param['name'],
                           "Wrong syntax used for setting up std::map<... , ...>: You provided '%s' with "
                           "parameter %s" % in_type)
                ptype = ptype[1].strip()
                if ptype == "std::string":
                    eprint(param['name'], "Max and min can not be specified for variable of type %s" % in_type)

        pattern = r'^[a-zA-Z][a-zA-Z0-9_]*$'
        if not re.match(pattern, param['name']):
            eprint(param['name'], "The name of field does not follow the ROS naming conventions, "
                                  "see http://wiki.ros.org/ROS/Patterns/Conventions")
        if param['configurable'] and (
                            param['global_scope'] or param['is_vector'] or param['is_map'] or param['constant']):
            eprint(param['name'],
                   "Global Parameters, vectors, maps and constant params can not be declared configurable! ")
        if param['global_scope'] and param['default'] is not None:
            eprint(param['name'], "Default values for global parameters should not be specified in node! ")
        if param['constant'] and param['default'] is None:
            eprint(param['name'], "Constant parameters need a default value!")
        if param['name'] in [p['name'] for p in self.parameters]:
            eprint(param['name'], "Parameter with the same name exists already")
        if param['edit_method'] == '':
            param['edit_method'] = '""'
        elif param['edit_method'] != '""':
            param['configurable'] = True

        # Check type
        if param['is_vector']:
            ptype = in_type[12:-1].strip()
            self._test_primitive_type(param['name'], ptype)
            param['type'] = 'std::vector<{}>'.format(ptype)
        elif param['is_map']:
            ptype = in_type[9:-1].split(',')
            if len(ptype) != 2:
                eprint(param['name'], "Wrong syntax used for setting up std::map<... , ...>: You provided '%s' with "
                                      "parameter %s" % in_type)
            ptype[0] = ptype[0].strip()
            ptype[1] = ptype[1].strip()
            if ptype[0] != "std::string":
                eprint(param['name'], "Can not setup map with %s as key type. Only std::map<std::string, "
                                      "...> are allowed" % ptype[0])
            self._test_primitive_type(param['name'], ptype[0])
            self._test_primitive_type(param['name'], ptype[1])
            param['type'] = 'std::map<{},{}>'.format(ptype[0], ptype[1])
        else:
            # Pytype and defaults can only be applied to primitives
            self._test_primitive_type(param['name'], in_type)
            param['pytype'] = self._pytype(in_type)

    @staticmethod
    def _pytype(drtype):
        """Convert C++ type to python type"""
        return {'std::string': "str", 'int': "int", 'double': "double", 'bool': "bool"}[drtype]

    @staticmethod
    def _test_primitive_type(name, drtype):
        """
        Test whether parameter has one of the accepted C++ types
        :param name: Parametername
        :param drtype: Typestring
        :return:
        """
        primitive_types = ['std::string', 'int', 'bool', 'float', 'double']
        if drtype not in primitive_types:
            raise TypeError("'%s' has type %s, but allowed are: %s" % (name, drtype, primitive_types))

    @staticmethod
    def _get_cvalue(param, field):
        """
        Helper function to convert strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        value = param[field]
        if param['type'] == 'std::string':
            value = '"{}"'.format(param[field])
        elif param['type'] == 'bool':
            value = str(param[field]).lower()
        return str(value)

    @staticmethod
    def _get_pyvalue(param, field):
        """
        Helper function to convert strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        value = param[field]
        if param['type'] == 'std::string':
            value = '"{}"'.format(param[field])
        elif param['type'] == 'bool':
            value = str(param[field]).capitalize()
        return str(value)

    @staticmethod
    def _get_cvaluelist(param, field):
        """
        Helper function to convert python list of strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        values = param[field]
        assert (isinstance(values, list))
        form = ""
        for value in values:
            if param['type'] == 'std::vector<std::string>':
                value = '"{}"'.format(value)
            elif param['type'] == 'std::vector<bool>':
                value = str(value).lower()
            else:
                value = str(value)
            form += value + ','
        # remove last ','
        return form[:-1]

    @staticmethod
    def _get_cvaluedict(param, field):
        """
        Helper function to convert python dict of strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        values = param[field]
        assert (isinstance(values, dict))
        form = ""
        for key, value in values.items():
            if param['type'] == 'std::map<std::string,std::string>':
                pair = '{{"{}","{}"}}'.format(key, value)
            elif param['type'] == 'std::map<std::string,bool>':
                pair = '{{"{}",{}}}'.format(key, str(value).lower())
            else:
                pair = '{{"{}",{}}}'.format(key, str(value))
            form += pair + ','
        # remove last ','
        return form[:-1]

    def generate(self, pkgname, nodename, classname):
        """
        Main working Function, call this at the end of your .params file!
        :param self:
        :param pkgname: Name of the catkin package
        :param nodename: Name of the Node that will hold these params
        :param classname: This should match your file name, so that cmake will detect changes in config file.
        :return: Exit Code
        """
        self.pkgname = pkgname
        self.nodename = nodename
        self.classname = classname

        if self.parent:
            eprint("You should not call generate on a group! Call it on the main parameter generator instead!")

        return self._generateImpl()

    def _generateImpl(self):
        """
        Implementation level function. Can be overwritten by derived classes.
        :return:
        """
        self._generatecfg()
        self._generatehpp()
        self._generatepy()

        return 0

    def _generatecfg(self):
        """
        Generate .cfg file for dynamic reconfigure
        :param self:
        :return:
        """
        templatefile = os.path.join(self.dynconfpath, "templates", "ConfigType.h.template")
        with open(templatefile, 'r') as f:
            template = f.read()

        param_entries = self._generate_param_entries()

        param_entries = "\n".join(param_entries)
        template = Template(template).substitute(pkgname=self.pkgname, nodename=self.nodename,
                                                 classname=self.classname, params=param_entries)

        cfg_file = os.path.join(self.share_dir, "cfg", self.classname + ".cfg")
        try:
            if not os.path.exists(os.path.dirname(cfg_file)):
                os.makedirs(os.path.dirname(cfg_file))
        except OSError:
            # Stupid error, sometimes the directory exists anyway
            pass
        with open(cfg_file, 'w') as f:
            f.write(template)
        os.chmod(cfg_file, 509)  # entspricht 775 (octal)

    def _generatehpp(self):
        """
        Generate C++ Header file, holding the parameter struct.
        :param self:
        :return:
        """

        # Read in template file
        templatefile = os.path.join(self.dynconfpath, "templates", "Parameters.h.template")
        with open(templatefile, 'r') as f:
            template = f.read()

        param_entries = []
        string_representation = []
        from_server = []
        to_server = []
        non_default_params = []
        from_config = []
        to_config = []
        test_limits = []

        params = self._get_parameters()

        # Create dynamic parts of the header file for every parameter
        for param in params:
            name = param['name']

            # Adjust key for parameter server
            if param["global_scope"]:
                namespace = 'globalNamespace'
            else:
                namespace = 'privateNamespace'
            full_name = '{} + "{}"'.format(namespace, param["name"])

            # Test for default value
            if param["default"] is None:
                default = ""
                non_default_params.append(Template('      << "\t" << $namespace << "$name" << " ($type) '
                                                   '\\n"\n').substitute(
                    namespace=namespace, name=name, type=param["type"]))
            else:
                if param['is_vector']:
                    default = ', {}'.format(str(param['type']) + "{" + self._get_cvaluelist(param, "default") + "}")
                elif param['is_map']:
                    default = ', {}'.format(str(param['type']) + "{" + self._get_cvaluedict(param, "default") + "}")
                else:
                    default = ', {}'.format(str(param['type']) + "{" + self._get_cvalue(param, "default") + "}")

            # Test for constant value
            if param['constant']:
                param_entries.append(Template('  static constexpr auto ${name} = $default; /*!< ${description} '
                                              '*/').substitute(type=param['type'], name=name,
                                                               description=param['description'],
                                                               default=self._get_cvalue(param, "default")))
                from_server.append(Template('    rosparam_handler::testConstParam($paramname);').substitute(paramname=full_name))
            else:
                param_entries.append(Template('  ${type} ${name}; /*!< ${description} */').substitute(
                    type=param['type'], name=name, description=param['description']))
                from_server.append(Template('    success &= rosparam_handler::getParam($paramname, $name$default);').substitute(
                    paramname=full_name, name=name, default=default, description=param['description']))
                to_server.append(
                    Template('  rosparam_handler::setParam(${paramname},${name});').substitute(paramname=full_name, name=name))

            # Test for configurable params
            if param['configurable']:
                from_config.append(Template('    $name = config.$name;').substitute(name=name))
                to_config.append(Template('   config.$name = $name;').substitute(name=name))

            # Test limits
            if param['is_vector']:
                ttype = param['type'][12:-1].strip()
            elif param['is_map']:
                ttype = param['type'][9:-1].strip()
            else:
                ttype = param['type']
            if param['min'] is not None:
                test_limits.append(Template('    rosparam_handler::testMin<$type>($paramname, $name, $min);').substitute(
                    paramname=full_name, name=name, min=param['min'], type=ttype))
            if param['max'] is not None:
                test_limits.append(Template('    rosparam_handler::testMax<$type>($paramname, $name, $max);').substitute(
                    paramname=full_name, name=name, max=param['max'], type=ttype))

            # Add debug output
            if param['is_vector'] or param['is_map']:
                string_representation.append(Template('      << "\t" << p.$namespace << "$name:" << rosparam_handler::to_string(p.$name) << '
                                                      '"\\n"\n').substitute(namespace=namespace, name=name))
            else:
                string_representation.append(Template('      << "\t" << p.$namespace << "$name:" << p.$name << '
                                                      '"\\n"\n').substitute(namespace=namespace, name=name))

        param_entries = "\n".join(param_entries)
        string_representation = "".join(string_representation)
        non_default_params = "".join(non_default_params)
        from_server = "\n".join(from_server)
        to_server = "\n".join(to_server)
        from_config = "\n".join(from_config)
        to_config = "\n".join(to_config)
        test_limits = "\n".join(test_limits)

        content = Template(template).substitute(pkgname=self.pkgname, ClassName=self.classname,
                                                parameters=param_entries, fromConfig=from_config,
                                                fromParamServer=from_server,
                                                string_representation=string_representation,
                                                non_default_params=non_default_params, nodename=self.nodename,
                                                test_limits=test_limits, toParamServer=to_server,
                                                toConfig=to_config)

        header_file = os.path.join(self.cpp_gen_dir, self.classname + "Parameters.h")
        try:
            if not os.path.exists(os.path.dirname(header_file)):
                os.makedirs(os.path.dirname(header_file))
        except OSError:
            # Stupid error, sometimes the directory exists anyway
            pass
        with open(header_file, 'w') as f:
            f.write(content)

    def _generatepy(self):
        """
        Generate Python parameter file
        :param self:
        :return:
        """
        params = self._get_parameters()
        paramDescription = str(params)

        # Read in template file
        templatefile = os.path.join(self.dynconfpath, "templates", "Parameters.py.template")
        with open(templatefile, 'r') as f:
            template = f.read()

        content = Template(template).substitute(pkgname=self.pkgname, ClassName=self.classname,
                                                paramDescription=paramDescription)

        py_file = os.path.join(self.py_gen_dir, "param", self.classname + "Parameters.py")
        try:
            if not os.path.exists(os.path.dirname(py_file)):
                os.makedirs(os.path.dirname(py_file))
        except OSError:
            # Stupid error, sometimes the directory exists anyway
            pass
        with open(py_file, 'w') as f:
            f.write(content)
        init_file = os.path.join(self.py_gen_dir, "param", "__init__.py")
        with open(init_file, 'wa') as f:
            f.write("")

    def _generateyml(self):
        """
        Generate .yaml file for roslaunch
        :param self:
        :return:
        """
        params = self._get_parameters()

        content = "### This file was generated using the rosparam_handler generate_yaml script.\n"

        for entry in params:
            if not entry["constant"]:
                content += "\n"
                content += "# Name:\t" + str(entry["name"]) + "\n"
                content += "# Desc:\t" + str(entry["description"]) + "\n"
                content += "# Type:\t" + str(entry["type"]) + "\n"
                if entry['min'] or entry['max']:
                    content += "# [min,max]:\t[" + str(entry["min"]) + "/" + str(entry["max"]) + "]" + "\n"
                if entry["global_scope"]:
                    content += "# Lives in global namespace!\n"
                if entry["default"] is not None:
                    content += str(entry["name"]) + ": " + str(entry["default"]) + "\n"
                else:
                    content += str(entry["name"]) + ": \n"

        yaml_file = os.path.join(os.getcwd(), self.classname + "Parameters.yaml")

        with open(yaml_file, 'w') as f:
            f.write(content)

    def _get_parameters(self):
        """
        Returns parameter of this and all childs
        :return: list of all parameters
        """
        params = self.parameters
        for child in self.childs:
            params.extend(child._get_parameters())
        return params

    def _generate_param_entries(self):
        """
        Generates the entries for the cfg-file
        :return: list of param entries as string
        """
        param_entries = []
        dynamic_params = [p for p in self.parameters if p["configurable"]]

        if self.parent:
            param_entries.append(Template("$group_variable = $parent.add_group('$group')").substitute(
                group_variable=self.group_variable,
                group=self.group,
                parent=self.parent.group_variable))

        for enum in self.enums:
            param_entries.append(Template("$name = gen.enum([").substitute(
                name=enum['name'],
                parent=self.group))
            i = 0
            for value in enum['values']:
                param_entries.append(
                    Template("    gen.const(name='$name', type='$type', value=$value, descr='$descr'),")
                        .substitute(name=value, type="int", value=i, descr=""))
                i += 1
            param_entries.append(Template("    ], '$description')").substitute(description=enum["description"]))

        for param in dynamic_params:
            content_line = Template("$group_variable.add(name = '$name', paramtype = '$paramtype', level = $level, "
                                    "description = '$description', edit_method=$edit_method").substitute(
                group_variable=self.group_variable,
                name=param["name"],
                paramtype=param['pytype'],
                level=param['level'],
                edit_method=param['edit_method'],
                description=param['description'])
            if param['default'] is not None:
                content_line += Template(", default=$default").substitute(default=self._get_pyvalue(param, "default"))
            if param['min'] is not None:
                content_line += Template(", min=$min").substitute(min=param['min'])
            if param['max'] is not None:
                content_line += Template(", max=$max").substitute(max=param['max'])
            content_line += ")"
            param_entries.append(content_line)

        for child in self.childs:
            param_entries.extend(child._generate_param_entries())
        return param_entries

    @staticmethod
    def _make_bool(param):
        if isinstance(param, bool):
            return param
        else:
            # Pray and hope that it is a string
            return bool(param)


# Create derived class for yaml generation
class YamlGenerator(ParameterGenerator):
    def _generateImpl(self):
        self._generateyml()
        return 0
