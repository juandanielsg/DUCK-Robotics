#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('/goal_pose', Float64MultiArray, queue_size=10)
    rospy.init_node('talker2', anonymous=True)
    rate = rospy.Rate(100)
    msg = Float64MultiArray()

    print("~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~")
    print("Publisher to set pose speeds.")
    print("Controls: ")
    print("W,A,S,D: +X, -Y, -X, +Y")
    print("Z,X: +Z, -Z")
    print("O,P: +R, -R")
    print("K,L: +P, -P")
    print("N,M: +Yaw, -Yaw")
    print("Spacebar: STOP")
    print("~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~")

    X=0
    Y=0
    Z=0
    R=0
    P=0
    Yaw=0

    while not rospy.is_shutdown():

        myinp = input()
        
        if myinp == "w":
            X += 0.01
        
        if myinp == "a":
            Y -= 0.01
        
        if myinp == "s":
            X -= 0.01

        if myinp == "d":
            Y += 0.01

        if myinp == "z":
            Z += 0.01

        if myinp == "c":
            Z -= 0.01
            
        if myinp == "o":
            R += 0.01

        if myinp == "p":
            R -= 0.01

        if myinp == "k":
            P += 0.01

        if myinp == "l":
            P -= 0.01

        if myinp == "n":
            Yaw += 0.01

        if myinp == "m":
            Yaw -= 0.01

        if myinp == "space":
            X = Y = Z = R = P = Yaw = 0

        msg.data = [X,Y,Z,R,P,Y]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass