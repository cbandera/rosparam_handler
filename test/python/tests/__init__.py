from os.path import dirname, basename, isfile
import glob
# automatically import all files in this module
modules = glob.glob(dirname(__file__)+"/*.py")
__all__ = [ basename(f)[:-3] for f in modules if isfile(f) and not f.endswith('__init__.py')]


# determine the test classes
def get_unittest_classes():
    """
    Determines the unittests to be passed to rostest
    :return:
    """
    import unittest
    unittests = {}
    for test_module in __all__:
        module = __import__("tests."+test_module, fromlist=['*'])
        for name in dir(module):
            obj = getattr(module, name)
            if isinstance(obj, type) and issubclass(obj, unittest.TestCase):
                unittests[name] = obj
    return unittests
