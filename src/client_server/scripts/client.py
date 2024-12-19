#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolRequest

rospy.init_node("client_node")

client = rospy.ServiceProxy("test_service", SetBool)

client.wait_for_service()

request = SetBoolRequest()
while not rospy.is_shutdown():
    input_as_string = input("select value: ")
    if input_as_string == "True" or input_as_string == "y" or input_as_string =="1":
        input_as_bool = True
    else:
        input_as_bool = False

    request.data = input_as_bool

    client.call(request)

rospy.spin()