#!/usr/bin/env python

PKG = 'rosparam_handler'
import unittest
import rospy
import tests

if __name__ == '__main__':
    import rostest
    unittests = tests.get_unittest_classes()
    rospy.init_node(PKG)
    for test_name, test in unittests.items():
        rostest.rosrun(PKG, test_name, test)
