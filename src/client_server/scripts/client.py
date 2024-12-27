#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolRequest

rospy.init_node("client_node")

client = rospy.ServiceProxy("test_service", SetBool)

client.wait_for_service()

request = SetBoolRequest()
while not rospy.is_shutdown():
    input_as_string = input("Input \"enable\" for enabling, \"disable\" for disabling the window (q for quit): ")
    if input_as_string == "q":
    	break
    elif input_as_string == "enable":
        input_as_bool = True
    elif input_as_string == "disable":
        input_as_bool = False
    else:
    	print("please enter a valid argument")
    	continue

    request.data = input_as_bool

    client.call(request)

