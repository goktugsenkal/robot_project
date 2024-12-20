#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64, Char, Byte, String


def topic_callback(msg):
    print(msg)

rospy.init_node("subscriber_node")


sub = rospy.Subscriber("test", String, callback=topic_callback)
sub_2 = rospy.Subscriber("test_2", String, callback=topic_callback)
rospy.spin()