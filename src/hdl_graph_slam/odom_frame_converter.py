#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause

import rospy
from nav_msgs.msg import Odometry

class OdomFrameConverter:

    def __init__(self):
        rospy.init_node('odom_frame_converter',anonymous=True)
        self.pub = rospy.Publisher('multijackal_02/odom_fixed',Odometry,queue_size=10)
        self.sub = rospy.Subscriber('multijackal_02/odom',Odometry,self.callback)
    
    def callback(self,data):
        msg = data
        msg.header.frame_id = 'multijackal_02/odom'
        msg.child_frame_id = 'multijackal_02/base_footprint'
        self.pub.publish(msg)

if __name__ == '__main__':
    converter = OdomFrameConverter()
    while not rospy.is_shutdown():
        rospy.spin()