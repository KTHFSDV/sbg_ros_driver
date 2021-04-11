#!/usr/bin/env python

import unittest
import rospy

from fs_msgs.msg import SlamState
from nav_msgs.msg import Odometry



class TestCommons(unittest.TestCase):

    def setUp(self): 

        self.lap_count = 0
        self.slam_odom = Odometry() 
        self.gt_odom = Odometry()

        # Init subscribers to retrieve commonly used information 
        
        rospy.Subscriber('/slam/slam/slam_state', SlamState, self._slam_state_cb)
        rospy.Subscriber('/slam/slam/odom', Odometry, self._slam_odom_cb)
        rospy.Subscriber('/sensors/odom/ground_truth', Odometry, self._gt_odom_cb)

    def tearDown(self):
        # Do some proper log here in the future (maybe of some relevant variables) 
        rospy.loginfo('Tearing down test, exiting....')

    def _slam_state_cb(self, data):
        self.lap_count = data.lap_counter

    def _slam_odom_cb(self, data):
        self.slam_odom = data

    def _gt_odom_cb(self, data):
        self.gt_odom = data







