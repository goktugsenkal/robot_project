#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

rospy.init_node("control")

pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

def callback(msg):
    print(msg)

rospy.Subscriber("/turtle1/pose", Pose, callback)

msg = Twist()

while not rospy.is_shutdown():
    msg.linear.x = random.randint(0,5)
    msg.linear.y = random.randint(0,5)
    
    pub.publish(msg)
    rospy.sleep(0.1)

rospy.spin()