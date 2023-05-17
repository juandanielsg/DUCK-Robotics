#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import math
import scipy
from math import sin, cos, atan2, sqrt
import os

import time
import datetime

#https://github.com/dqrobotics/python
#Have a look at this.

#YOLO
#import quaternion

from Utility import build_se3_transform, euler_to_so3, so3_to_euler

#TODO NOW

#Add a self.goal part and a Subscriber to a certain topic with custom Goal msg
#Add a new callback to said goal, which, when activated:
#1. Sets self.goal to the new goal received
#2. Triggers a SLERP function to generate some intermediate goals stored in a vector of self.subgoals
#3. Make the robot follow the subgoals in order
#4. Reach the final goal on time


#Do the same with DUAL quaternions... This should be faster, you know?
#Please God, have mercy

#Imagine not using these... What a dumb*ss
from dual_quaternions import DualQuaternion

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return qw, qx, qy, qz

def euler_from_quaternion(w, x, y, z):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return [roll_x, pitch_y, yaw_z] # in radians


class UR10e():

    #Now with DQ. How fun :D

    def __init__(self, logfile=None):

        # Do something on shutdown
        rospy.on_shutdown(self.onShutdown)

        # K value
        self.K = 1
        
        # DH Params
        self.d1 =  0.1807
        self.d2 = 0
        self.d3 = 0
        self.d4 =  0.17415
        self.d5 =  0.11985
        self.d6 =  0.11655

        self.d = [self.d1, self.d2, self.d3, self.d4, self.d5, self.d6]

        self.a1 = 0
        self.a2 = -0.6127
        self.a3 = -0.57155
        self.a4 = 0
        self.a5 = 0
        self.a6 = 0

        self.a = [self.a1, self.a2, self.a3, self.a4, self.a5, self.a6]
    
        self.alpha1 = math.pi/2
        self.alpha2 = 0
        self.alpha3 = 0
        self.alpha4 = math.pi/2
        self.alpha5 = -math.pi/2
        self.alpha6 = 0

        self.alpha = [self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6]

        #ROS publishers/subscribers
        self.poseSub = rospy.Subscriber("/joint_states", JointState, self.callback)
        self.velPub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        #Current link positions
        self.an3, self.an2, self.an1, self.an4, self.an5, self.an6 = None, None, None, None, None, None
        self.v3, self.v2, self.v1, self.v4, self.v5, self.v6 = None, None, None, None, None, None

        self.an = []
        self.v = []

        #Maximum joint positions and speed
        self.clamp_pose = [-np.pi*2, np.pi*2]
        #self.clamp_speed_1 = [-2*np.pi/3, 2*np.pi/3]
        #self.clamp_speed_2 = [-np.pi, np.pi]

        self.clamp_speed_1 = [-0.1, 0.1]
        self.clamp_speed_2 = [-0.1, 0.1]

        #Homog. Transform Matrices for Forward Kinematics
        self.DQ0 = None
        self.DQ1 = None
        self.DQ2 = None
        self.DQ3 = None
        self.DQ4 = None
        self.DQ5 = None
        self.DQ6 = None
        self.setRefDQ()

        self.DQ_list = []

        #Rot and displacement of end effector
        self.F_DQ = None
        self.R = None
        self.t = None

        #Poses
        self.pose1 = None
        self.pose2 = None
        self.pose3 = None
        self.pose4 = None
        self.pose5 = None
        self.pose6 = [None, None, None, None, None, None]

        #Jacobian
        self.J = np.array([[1,0],[1,0]])
        self.J_inv = None

        #Current Speed
        self.jointSpeed = [None]*6

        #Current Goal
        self.jointGoal = [None]*6

        #PID Params
        self.Kp = 0.9
        self.Ki = 0.1
        self.Kd = 0.1

        self.dt = 0.001

        self.P = None
        self.I = None
        self.D = None

        self.prev_e = 0
        self.sum_e = 0

        #Generate a new logfile to add my stuff - WIP not done yet
        self.logfilename = logfile
        self.initFile()

        #Please God, be merciful
        self.error = None
        self.num_error = None

        #Pose Goal
        self.goal = [None, None, None, None, None, None]
        self.goalDQ = None

    def initFile(self):
        pass

    def printPose(self, time):
        print("~ Average iteration time ~")
        print(str(time) + " s")
        self.dt = time
        print()
        print("~ POSE ~")
        print("X: ", self.pose6[0])
        print("Y: ", self.pose6[1])
        print("Z: ", self.pose6[2])
        print("R: ", self.pose6[3])
        print("P: ", self.pose6[4])
        print("Y: ", self.pose6[5])
        print()
        """
        print("~ SPEED ~")
        print("shoulder_pan_joint: ", self.jointSpeed[0])
        print("shoulder_lift_joint: ", self.jointSpeed[1])
        print("elbow_joint: ", self.jointSpeed[2])
        print("wrist_1_joint: ", self.jointSpeed[3])
        print("wrist_2_joint: ", self.jointSpeed[4])
        print("wrist_3_joint: ", self.jointSpeed[5])
        """
        print()
        print("~ Sim Info ~")
        print("shoulder_pan_joint: ", self.an1, "\t",self.v1)
        print("shoulder_lift_joint: ", self.an2, "\t",self.v2)
        print("elbow_joint: ", self.an3, "\t",self.v3)
        print("wrist_1_joint: ", self.an4, "\t",self.v4)
        print("wrist_2_joint: ", self.an5, "\t",self.v5)
        print("wrist_3_joint: ", self.an6, "\t",self.v6)
        print()
        """
        print("~ Jacobian Matrix ~")
        print("Det(JJ_t): ", np.linalg.det(self.J @ self.J.T))
        print('\n'.join(['\t'.join([str(round(cell,3)) for cell in row]) for row in self.J]))
        print()
        print("~ Interpolation Info ~ ")
        print(self.tmpIndex, "/", self.maxIndex+1, " step.")
        if self.num_error is not None:
            print("Error (scalar): ", self.num_error)

        if self.error is not None:
            print()
            print("~ Error ~")
            print('\t'.join([str(round(cell,4)) for cell in self.error]))
            print("P: " + str(self.P))
            print("I: " + str(self.I))
            print("D: " + str(self.D))
        
        """

    def setRefDQ(self):
        w, x, y, z = get_quaternion_from_euler(0, 0, 180)
        print(w,x,y,z)
        self.DQ0 = DualQuaternion.from_quat_pose_array([w,x,y,z,0,0,0])
        
    def getDQ(self,index):

        index -= 1

        q_rw = cos(self.alpha[index]/2)*cos(self.an[index]/2)
        q_rx = sin(self.alpha[index]/2)*cos(self.an[index]/2)
        q_ry = sin(self.alpha[index]/2)*sin(self.an[index]/2)
        q_rz = cos(self.alpha[index]/2)*sin(self.an[index]/2)
        q_tw = (-1/2) * self.a[index] * sin(self.alpha[index]/2) * cos(self.an[index]/2) - ((1/2) * self.d[index] * cos(self.alpha[index]/2) * sin(self.an[index]/2))
        q_tx = (1/2) * self.a[index] * cos(self.alpha[index]/2) * cos(self.an[index]/2) - ((1/2) * self.d[index] * sin(self.alpha[index]/2) * sin(self.an[index]/2))
        q_ty = (1/2) * self.a[index] * cos(self.alpha[index]/2) * sin(self.an[index]/2) + (1/2) * self.d[index] * sin(self.alpha[index]/2) * cos(self.an[index]/2)
        q_tz = (1/2) * self.d[index] * cos(self.alpha[index]/2) * cos(self.an[index]/2) - ((1/2) * self.a[index] * sin(self.alpha[index]/2) * sin(self.an[index]/2))

        return DualQuaternion.from_dq_array([q_rw, q_rx, q_ry, q_rz, q_tw, q_tx, q_ty, q_tz])

    def getAllDQ(self):

        self.DQ1 = self.getDQ(1)
        self.DQ2 = self.getDQ(2)
        self.DQ3 = self.getDQ(3)
        self.DQ4 = self.getDQ(4)
        self.DQ5 = self.getDQ(5)
        self.DQ6 = self.getDQ(6)

    

    def FK(self):

        self.F_DQ = self.DQ0 * self.DQ1 * self.DQ2 * self.DQ3 * self.DQ4 * self.DQ5 * self.DQ6

        H = self.F_DQ.homogeneous_matrix()

        P = atan2(-H[2,0],sqrt(H[2,1]**2+H[2,2]**2))
        R = atan2(H[2,1]/cos(P), H[2,2]/cos(P))
        Y = atan2(H[1,0]/cos(P), H[0,0]/cos(P))

        self.pose6 = [H[0,3], H[1,3], H[2,3], R, P, Y]



        #quatpose = DualQuaternion.quat_pose_array(self.F_DQ)

        #R, P, Y = euler_from_quaternion(quatpose[0], quatpose[1], quatpose[2], quatpose[3])
        #self.pose6 = [quatpose[3], quatpose[4], quatpose[5], R, P, Y]


    def callback(self,msg):

        #This works fine

        self.an3, self.an2, self.an1, self.an4, self.an5, self.an6 = msg.position
        self.v3, self.v2, self.v1, self.v4, self.v5, self.v6 = msg.velocity

        self.an = [self.an1, self.an2, self.an3, self.an4, self.an5, self.an6]
        self.v = [self.v1, self.v2, self.v3, self.v4, self.v5, self.v6]

    def onShutdown(self):
        msg = Float64MultiArray()
        msg.data = [0,0,0,0,0,0]
        self.velPub.publish(msg)

    def setGoal(self,goal):

        pass

def main():
    now = datetime.datetime.now()
    nowstr = now.strftime("%H:%M:%S_%d/%m/%y")
    logname = "log_"+nowstr+".txt"
    rospy.init_node("CartVelCntrl")
    robot = UR10e(logfile=logname)
    cnt = 0
    ref_rate = 500
    time1 = time.time()

    goalPose = [0.9, 0, 0.6, 0.4, 0, 0.2]
    #robot.setGoal(goalPose, interp=False)

    while not rospy.is_shutdown():
        
        if robot.an1 is not None:

            robot.getAllDQ()
            robot.FK()
            #robot.calculateJ()
            #robot.reachPose()
        
        cnt += 1
        if cnt % ref_rate == 0:
            os.system("clear")
            robot.printPose((time.time()-time1)/ref_rate)
            time1 = time.time()
            pass

    


if __name__ == "__main__":
    main()