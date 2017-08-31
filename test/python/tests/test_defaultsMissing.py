import unittest
from rosparam_handler.param.DefaultsMissingParameters import DefaultsMissingParameters


class TestDefaultsMissingParameters(unittest.TestCase):
    def runTest(self):
        with self.assertRaises(KeyError):
            params = DefaultsMissingParameters()
