#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from immortals_messages.msg import EulerPoseAndFlags
from geometry_msgs.msg import Twist
import numpy as np
import math
import scipy
from math import sin, cos, atan2, sqrt
import os

import time
import datetime

def talker(pub):

    rospy.init_node('talker', anonymous=True)
    
    msg = Twist()

    print("~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~")
    print("Publisher to set goals.")
    print("All goals must be specified as X,Y,Z,R,P,Y.")
    print("Commas are really important.")
    print("~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~")

    while not rospy.is_shutdown():
        
        data = str(input("Set the desired goal, separated by commas: "))

        data = [float(datum) for datum in data.split(",")]
        print(data)
        
        try:

            msg.linear.x = data[0]
            msg.linear.y = data[1]
            msg.linear.z = data[2]
            msg.angular.x = data[3]
            msg.angular.y = data[4]
            msg.angular.z = data[5]

            print(msg)

            if len(data) == 6:

                print("X: ", data[0])
                print("Y: ", data[1])
                print("Z: ", data[2])
                print("R: ", data[3])
                print("P: ", data[4])
                print("Y: ", data[5])
            
            else:

                print("Error: wrong format")
        
            pub.publish(msg)
        
        except:

            print("Error: wrong format")

if __name__ == '__main__':
    try:
        controller = rospy.get_param('controller_type')
        if controller == "cartV" or controller == "jointV":
            pub = rospy.Publisher('/goal_speed', Twist, queue_size=10)
        
        else:
            pub = rospy.Publisher('/goal_pose', Twist, queue_size=10)
        talker(pub)

    except rospy.ROSInterruptException:
        pass