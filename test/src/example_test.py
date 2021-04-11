#!/usr/bin/env python


PKG='test'
NAME='example_test'


import unittest
import rospy
from fs_msgs.msg import SlamState
from test_commons import TestCommons


class ExampleTest(TestCommons):
    
    def setUp(self):
        super(ExampleTest, self).setUp()

        self.start_time = rospy.get_time()
        self.rate = rospy.Rate(10)

        # Defined in launch file of test, should have same value as the node time-limit
        self.time_limit = rospy.get_param('/my_ex_test/time-limit')
        

    def tearDown(self):
        super(ExampleTest, self).tearDown()

    def test_thing(self):
        """
            Test that the estimated position of the car is within 
            an epsilon distance from the ground truth
        """

        # Wait for some condition 
        while self.lap_count < 1: 
            self.rate.sleep()

        # Then do test 
        est = self.slam_odom.pose.pose.position
        gt = self.gt_odom.pose.pose.position

        x_diff = abs(est.x - gt.x)
        y_diff = abs(est.y - gt.y)
        z_diff = abs(est.z - gt.z)
        
        eps = 10

        self.assertTrue(x_diff < eps)
        # Running checks more than once:

        # Run as long as possible

        # while not rospy.is_shutdown() and rospy.get_time() - self.start_time < self.time_limit - 2:
        #     self.assertTrue(x_diff < eps, ('X pos difference between gt and estimated {:.5f}').format(x_diff))
        #     self.assertTrue(y_diff < eps, ('Y pos difference between gt and estimated {:.5f}').format(y_diff))
        #     self.assertTrue(z_diff < eps, ('Z pos difference between gt and estimated {:.5f}').format(z_diff))

        # Alternatively run test for x amount of seconds

        # time_limit = 30 # Seconds
        # frequency = 2 # twice a second, perform checks
        # self.rate = rospy.Rate(frequency)

        # for i in range(time_limit * frequency):
        #     self.assertTrue(True)
        #     self.rate.sleep()

if __name__ == '__main__':
    import rostest
    rospy.init_node(NAME, anonymous=True)

    rostest.rosrun(PKG, NAME, ExampleTest)

