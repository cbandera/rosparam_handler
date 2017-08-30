import unittest
from rosparam_handler.param.DefaultsParameters import DefaultsParameters
import rospy


class TestDefaults(unittest.TestCase):
    def test_defaults(self):
        params = DefaultsParameters()
        self.assertEqual(params.int_param_w_default, 1)
        self.assertAlmostEqual(params.double_param_w_default, 1.1)
        self.assertEqual(params.str_param_w_default, "Hello World")
        self.assertEqual(params.bool_param_w_default, True)

        self.assertEqual(params.vector_int_param_w_default, [1, 2, 3])
        self.assertEqual(params.vector_double_param_w_default, [1.1, 1.2, 1.3])
        self.assertEqual(params.vector_string_param_w_default, ["Hello", "World"])

        self.assertEqual(params.map_param_w_default, {"Hello": "World"})
        self.assertEqual(params.enum_param_w_default, 1)

    def test_set_defaults_on_param_server(self):
        params = DefaultsParameters()
        # now all parameters should be set on param server
        self.assertEqual(params.int_param_w_default, rospy.get_param("~int_param_w_default"))
        self.assertAlmostEqual(params.double_param_w_default, rospy.get_param("~double_param_w_default"))
        self.assertEqual(params.str_param_w_default, rospy.get_param("~str_param_w_default"))
        self.assertEqual(params.bool_param_w_default, rospy.get_param("~bool_param_w_default"))

        self.assertEqual(params.vector_int_param_w_default, rospy.get_param("~vector_int_param_w_default"))
        self.assertEqual(params.vector_double_param_w_default, rospy.get_param("~vector_double_param_w_default"))
        self.assertEqual(params.vector_string_param_w_default, rospy.get_param("~vector_string_param_w_default"))

        self.assertEqual(params.map_param_w_default, rospy.get_param("~map_param_w_default"))
        self.assertEqual(params.enum_param_w_default, rospy.get_param("~enum_param_w_default"))
