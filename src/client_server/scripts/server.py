#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

tabela_state = False

def callback(req):
    global tabela_state
    response = SetBoolResponse()
    if req.data is True:
        if tabela_state == False:
            tabela_state = True
            response.success = True
            response.message = "tabela is enabled"
        else:
            response.success = False
            response.message = "tabela already enabled!!"
    else:
        if tabela_state == True:
            tabela_state = False
            response.success = True
            response.message = "tabela is disabled"
        else:
            response.success = False
            response.message = "tabela already disabled!!"
    print(response)
    return response

rospy.init_node("server_node")

rospy.Service("test_service", SetBool, callback)

rospy.spin()