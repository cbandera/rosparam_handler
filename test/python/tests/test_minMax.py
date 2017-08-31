import unittest
from rosparam_handler.param.MinMaxParameters import MinMaxParameters


class TestMinMaxParameters(unittest.TestCase):
    def test_min_max_parameters(self):
        params = MinMaxParameters()
        self.assertEqual(params.int_param_w_minmax, 2)
        self.assertAlmostEqual(params.double_param_w_minmax, 2.)

        self.assertEqual(params.vector_int_param_w_minmax, [0, 2, 2])
        self.assertEqual(params.vector_double_param_w_minmax, [0, 1.2, 2.])

        self.assertEqual(params.map_param_w_minmax, {"value1": 0., "value2": 1.2, "value3": 2.})
