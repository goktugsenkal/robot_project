#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64, Char, Byte, String


def topic_callback(msg):
    print(msg)

rospy.init_node("subscriber_node")

sub = rospy.Subscriber("topic", String, callback=topic_callback)

rospy.spin()