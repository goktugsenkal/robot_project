#!/usr/bin/env python3

import rospy
import rosbag

rospy.init_node("read_bag")

bag = rosbag.Bag("/home/goktug/catkin_test_ws/src/pub_sub/bag/test.bag")

for topic, msg, t in bag.read_messages(topics=["/test", "/test2"]):
    if topic == "/test":
        print("Test message is: ")
        print(msg.data)
        print("And it was recorded at: ")
        print(t)
    if topic == "/test2":
        print("Test2 message is: ")
        print(msg.data)
        print("And it was recorded at: ")
        print(t)