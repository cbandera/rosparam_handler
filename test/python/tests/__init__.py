from os.path import dirname, basename, isfile
import glob
import unittest
# automatically import all files in this module
modules = glob.glob(dirname(__file__)+"/*.py")
__all__ = [ basename(f)[:-3] for f in modules if isfile(f) and not f.endswith('__init__.py')]

class RosparamTestSuite(unittest.TestSuite):

        def __init__(self):
            super(RosparamTestSuite, self).__init__()

            # Collect test cases
            testcases = {}
            for test_module in __all__:
                module = __import__("tests."+test_module, fromlist=['*'])
                for name in dir(module):
                    obj = getattr(module, name)
                    if isinstance(obj, type) and issubclass(obj, unittest.TestCase):
                        testcases[name] = unittest.TestLoader().loadTestsFromTestCase(obj)

            # Add testcases
            for test_name, test in testcases.items():
                self.addTest(test)
