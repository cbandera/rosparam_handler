#!/usr/bin/env python

PKG = 'rosparam_handler'
import unittest
import rospy
import tests

if __name__ == '__main__':
    import rostest

    rospy.init_node(PKG)
    rostest.rosrun(PKG, "RosparamTestSuite", "tests.RosparamTestSuite")
