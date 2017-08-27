import unittest
from rosparam_handler.param.DefaultsMissingParameters import DefaultsMissingParameters


class TestDefaultsMissingParameters(unittest.TestCase):
    def test_defaults_missing_params(self):
        with self.assertRaises(KeyError):
            params = DefaultsMissingParameters()
