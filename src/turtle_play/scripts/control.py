#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
from colorama import Fore, Style, init

init(autoreset=True)

rospy.init_node("control")

pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

def callback(msg):
    print(
        f"\n{Fore.CYAN}{Style.BRIGHT}=== Turtle Pose ===\n"
        f"{Fore.GREEN}X: {msg.x:.2f}\n"
        f"Y: {msg.y:.2f}\n"
        f"Theta: {msg.theta:.2f}\n"
        f"Linear Velocity: {msg.linear_velocity:.2f}\n"
        f"Angular Velocity: {msg.angular_velocity:.2f}\n"
        f"{Fore.CYAN}==================={Style.RESET_ALL}"
    )

rospy.Subscriber("/turtle1/pose", Pose, callback)

msg = Twist()

while not rospy.is_shutdown():
    msg.linear.x = random.uniform(0, 5)
    msg.linear.y = random.uniform(0, 5)
    msg.angular.z = random.uniform(0, 5)
    
    pub.publish(msg)
    
    rospy.sleep(0.1)

