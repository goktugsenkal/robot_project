#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8, Char, Float32, String, UInt64MultiArray

# node & topic init
rospy.init_node("publisher_node")
pub = rospy.Publisher("test_topic_1", String, queue_size=1)
pub_2 = rospy.Publisher("test_topic_2", String, queue_size=1)

# publish loop
while not rospy.is_shutdown():
    pub.publish("Message to test topic 1.")
    pub_2.publish("Message to test topic 2")
    rospy.sleep(1)

