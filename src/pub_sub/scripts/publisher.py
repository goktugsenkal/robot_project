#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8, Char, Float32, String, UInt64MultiArray

# node & topic init
rospy.init_node("publisher_node")
pub = rospy.Publisher("test", String, queue_size=1)
pub_2 = rospy.Publisher("test_2", String, queue_size=1)

# publish loop
while not rospy.is_shutdown():
    pub.publish("wenk")
    pub_2.publish("wenk 2")
    rospy.sleep(1)

