#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8, Char, Float32, String, UInt64MultiArray

# node & topic init
rospy.init_node("publisher_node")
pub = rospy.Publisher("topic", String, queue_size=1)

# publish loop
while not rospy.is_shutdown():
    pub.publish("wenk" )
    rospy.sleep(1)

