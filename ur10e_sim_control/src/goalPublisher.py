#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from immortals_messages.msg import Pose
import numpy as np
import math
import scipy
from math import sin, cos, atan2, sqrt
import os

import time
import datetime

def talker():
    pub = rospy.Publisher('/goal_pose', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    msg = Pose()

    print("~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~")
    print("Publisher to set pose goals.")
    print("All poses must be specified as X,Y,Z,R,P,Y.")
    print("Commas are really important.")
    print("~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~")

    while not rospy.is_shutdown():
        
        data = str(input("Set the desired goal, separated by commas: "))

        data = [float(datum) for datum in data.split(",")]
        print(data)
        
        try:

            msg.x = data[0]
            msg.y = data[1]
            msg.z = data[2]
            msg.roll = data[3]
            msg.pitch = data[4]
            msg.yaw = data[5]
            msg.jointNumber = 6

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
        talker()
    except rospy.ROSInterruptException:
        pass