#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32

rospy.init_node('basic_motion', anonymous=True)
pub = rospy.Publisher('cmd_vel', Int32, queue_size=10)
rospy.sleep(1)
