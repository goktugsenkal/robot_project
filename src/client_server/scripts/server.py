#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

window_state = False

def callback(req):
    global window_state
    response = SetBoolResponse()
    if req.data is True:
        if window_state == False:
            window_state = True
            response.success = True
            response.message = "Operation is enabled"
        else:
            response.success = False
            response.message = "Operation already enabled!!"
    else:
        if window_state == True:
            window_state = False
            response.success = True
            response.message = "Operation is disabled"
        else:
            response.success = False
            response.message = "operation already disabled!!"
    print(response)
    if req.data == None:
    	response.success = False
    	response.message = "Invalid argument"
    return response

rospy.init_node("server_node")

rospy.Service("test_service", SetBool, callback)

rospy.spin()
