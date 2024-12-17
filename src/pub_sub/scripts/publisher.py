#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

rospy.init_node("publisher_node")

pub = rospy.Publisher("topic", Int64, )
