#!/usr/bin/env python
import os

import rospy
import rostest
from std_msgs.msg import Bool

from test_commons import TestCommons

class StopStartTest(TestCommons):
    
    def setUp(self):
        super(StopStartTest, self).setUp()
        
        self.rate = rospy.Rate(10)

        self.stop_pub = rospy.Publisher('/stop_car_request', Bool, queue_size=1)
        self.start_pub = rospy.Publisher('/start_car_request', Bool, queue_size=1)

    def test_stop(self):
        """
            Test that the car is able to come to a full stop within
            some time frame, and then restart again. The test is run
            during the second lap. 
            
        """
        while self.lap_count < 2: 
            self.rate.sleep()

        # Then do test 
        # Perhaps the ground truth should be used
        est = self.slam_odom.pose.pose.position
        
        self.stop_pub.publish(False)

        time_limit = 30
        frequency = 2
        has_stopped = False
        eps = 1e-1

        self.rate = rospy.Rate(frequency)
        
        for i in range(time_limit * frequency):
           s = self.slam_odom.twist.twist.linear.x 
           if -eps <= s <= eps:
               has_stopped = True
               break
           self.rate.sleep()

        self.assertTrue(has_stopped, ('The car did not come to a comlete stop within {:d} s'
            .format(time_limit)))

        self.start_pub.publish(True)
        
        restarted = False
        time_limit = 10
        
        for i in range(time_limit * frequency):
            s = self.slam_odom.twist.twist.linear.x
            if  abs(s) > 1: # idk
                restarted = True
                break
            self.rate.sleep()

        self.assertTrue(restarted, ('The car did not restart within {:d} s'.format(time_limit)))

if __name__ == '__main__':
    test_name = os.path.basename(__file__)
    rospy.init_node(test_name, anonymous=True)

    rostest.rosrun('test', test_name, StopStartTest)