#!/usr/bin/env python


PKG='test'
NAME='example_test'


import unittest
import rospy
from fs_msgs.msg import SlamState



class ExampleTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.Subscriber('/slam/slam/slam_state', SlamState, self._wait_for_lap)
        self.rate = rospy.Rate(10)

        self.test_ready = False
        # set up subscribers or w/e

    def tearDown(self):
        pass

    def _wait_for_lap(self, data):
        if data.lap_counter > 3:
            self.test_ready = True

    def test_thing(self):
        while not self.test_ready:
            self.rate.sleep()

        # Otherwise do test 
        self.assertEqual(1, 1)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, ExampleTest)

