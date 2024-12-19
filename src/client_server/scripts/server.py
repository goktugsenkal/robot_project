#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def callback(req):
    response = SetBoolResponse()
    if req.data is True:
        response.success = True
        response.message = "tabela is enabled"
    else:
        response.success = True
        response.message = "tabela is disabled"

    print(response)
    return response

rospy.init_node("server_node")

rospy.Service("test_service", SetBool, callback)

rospy.spin()