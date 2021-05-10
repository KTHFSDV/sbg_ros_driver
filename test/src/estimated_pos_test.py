#!/usr/bin/env python
"""
    Test if the estimated position of the car is 
    within some epsilon to what is expected 
    after three laps (i.e. the first point on the planned path).
"""

import os 

import rospy
import rostest
from fs_msgs.msg import PlannedPath

from test_commons import TestCommons

class EstimatedPosTest(TestCommons):
    
    def setUp(self):
        super(EstimatedPosTest, self).setUp()

        self.rate = rospy.Rate(10)

        self.init_path_pos = None  # Holds the first point on the path 
        rospy.Subscriber('/navigation/path_planner/path', PlannedPath, self._path_cb) 
        
    def _path_cb(self, data):
        """
            For this test we only need the initial position of the path 
        """
        if data.x and data.y: 
            self.init_path_pos = (data.x[0], data.y[0])

    def test_est_to_path(self):
        """
            Test that the estimated position of the car is within 
            an epsilon distance from the ground truth
        """

        # Run test when we reach lap 3 
        while self.lap_count < 3: 
            self.rate.sleep()

        # Then do test 
        est = self.slam_odom.pose.pose.position
        self.assertTrue(self.init_path_pos, 'NO PATH HAVE BEEN RECEIVED')

        x_diff = abs(est.x - self.init_path_pos[0])
        y_diff = abs(est.y - self.init_path_pos[1])
    
        # Compare estimated pose with expected pose on path        
        eps = 5

        self.assertTrue(x_diff < eps, ('X pos difference between estimated and expected {:.5f}').format(x_diff))
        self.assertTrue(y_diff < eps, ('Y pos difference between estimated and expected {:.5f}').format(y_diff))

if __name__ == '__main__':
    test_name = os.path.basename(__file__)
    rospy.init_node(test_name, anonymous=True)

    rostest.rosrun('test', test_name, EstimatedPosTest)