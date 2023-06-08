#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from immortals_messages.msg import PoseAndFlags
import numpy as np
import math
import scipy
from math import sin, cos, atan2, sqrt
import os

import time
import datetime

def talker(pub):

    

    rospy.init_node('talker', anonymous=True)
    
    msg = PoseAndFlags()
    #<include file="$(find ur_description)/launch/load_ur10e.launch" />
    #<include file="$(find ur_description)/launch/load_ur10e.launch"/>
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

            msg.pose.x = data[0]
            msg.pose.y = data[1]
            msg.pose.z = data[2]
            msg.pose.roll = data[3]
            msg.pose.pitch = data[4]
            msg.pose.yaw = data[5]
            msg.pose.jointNumber = 6

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
            
            
            plan = int(input("Type '1' to add path planning: "))
            if plan == 1:
                msg.plan = True
            else:
                msg.plan = False
            pub.publish(msg)
        
        except:

            print("Error: wrong format")

if __name__ == '__main__':
    try:
        controller = rospy.get_param('controller_type')
        if controller == "cartV" or controller == "jointV":
            pub = rospy.Publisher('/goal_speed', PoseAndFlags, queue_size=10)
        
        else:
            pub = rospy.Publisher('/goal_pose', PoseAndFlags, queue_size=10)
        talker(pub)

    except rospy.ROSInterruptException:
        pass