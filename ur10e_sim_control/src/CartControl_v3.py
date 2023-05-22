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
import quaternion
import pathlib

from ur10e_sim_control.Utility import get_quaternion_from_euler, euler_from_quaternion, KinematicChain

#TODO NOW
#Improve code
#Make it serial
#Standarize stuff

#"El que tenga miedo a morir que no nazca" - typical Spanish quote
#"Me cago en las farolas que alumbran la tumba de las familias de los creadores de KDL y UR" - average Spanish insult

class UR10e():

    def __init__(self, logfile=None, sim=True):

        # Do something on shutdown
        rospy.on_shutdown(self.onShutdown)

        # K value
        self.K = 10
        
        #Denavit-Hartenberg parameters - Now modifiable.
        self.d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
        self.a = [0, -0.6127, -0.57155, 0, 0, 0]
        self.alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]

        #ROS publishers/subscribers
        self.goalSub = rospy.Subscriber("/goal_pose", Pose, self.goalcallback, queue_size=1)
        self.poseSub = rospy.Subscriber("/joint_states", JointState, self.callback)
        self.velSub = rospy.Subscriber("/goal_speed", Float64MultiArray, self.speedCallback, queue_size=1)
        self.velPub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        #Current link positions
        self.an3, self.an2, self.an1, self.an4, self.an5, self.an6 = None, None, None, None, None, None
        self.v3, self.v2, self.v1, self.v4, self.v5, self.v6 = None, None, None, None, None, None

        self.an = []
        self.v = []

        #Maximum joint positions and speed - WIP. Add

        self.clamp_angle = [-np.pi*2, np.pi*2]

        self.clamp_speed_1 = [-0.01, 0.01]
        self.clamp_speed_2 = [-0.01, 0.01]


        # **NOVELTY** Added Kinematic chain to the system

        self.KinChain = KinematicChain()
        if sim:
            self.KinChain.setBase(self.messySolution())
        else:
            self.KinChain.setBase(np.eye(4))

        #Rot and displacement of end effector
        self.EEHomMat = None
        self.EErotation = None
        self.EEtranslation = None

        #Poses
        self.pose1 = None
        self.pose2 = None
        self.pose3 = None
        self.pose4 = None
        self.pose5 = None
        self.pose6 = None

        #Jacobian
        self.J = np.array([[1,0],[1,0]])
        self.J_inv = None

        #Current Speed
        self.jointSpeed = [None]*6

        #PID Params
        self.Kp = 0.9
        self.Ki = 0.1
        self.Kd = 0.1

        self.dt = 0.001

        self.P = None
        self.I = None
        self.D = None

        self.error = None
        self.prev_e = 0
        self.sum_e = 0

        #Generate a new logfile to collect data - WIP
        self.logfilename = logfile
        self.log = None
        self.initFile()

        #Please God, be merciful
        self.qt = [None, None, None, None]

        self.interpolations = []

        self.num_error = None

        #Pose Goal
        self.goal = None
        self.goalSpeed = None
        self.goalqt = [None, None, None, None]

        #Temporary goals
        self.tmpGoal = [None, None, None, None, None, None]
        self.tmpIndex = None
        self.maxIndex = None

    def initFile(self) -> None:
        """Creates a new log file in the logs/ directory to store data during the execution of the program. It will override any log files with the same name, but this should not happen.
        """

        self.log = open(self.logfilename, "w")

    def pose2String(self, pose) -> str:
        """Converts a pose (6 value array) into a string. This is a method called by logWrite() only, and it has no other use.

        Args:
            pose (np.array): a 6 value array in the form of [x,y,z,roll,pitch,yaw] which defines space poses in the Cartesian/Euler space.

        Returns:
            string: a string in the format x/y/z/roll/pitch/yaw with a line jump at the end.
        """
        
        return "/".join([str(coord) for coord in pose]) + "\n"
    
    def logWrite(self) -> None:
        
        data = [self.pose2String(self.pose1), self.pose2String(self.pose2), self.pose2String(self.pose3), self.pose2String(self.pose4), self.pose2String(self.pose5), self.pose2String(self.pose6), "\n"]

        self.log.writelines(data)

    def printPose(self, time) -> None:
        print("~ Average iteration time ~")
        print(str(time) + " s")
        self.dt = time
        print()
        if self.pose6 is not None:
            print("~ POSE ~")
            print("X: ", self.pose6[0])
            print("Y: ", self.pose6[1])
            print("Z: ", self.pose6[2])
            print("R: ", self.pose6[3])
            print("P: ", self.pose6[4])
            print("Y: ", self.pose6[5])
            print()
        print("~ SPEED ~")
        print("shoulder_pan_joint: ", self.jointSpeed[0])
        print("shoulder_lift_joint: ", self.jointSpeed[1])
        print("elbow_joint: ", self.jointSpeed[2])
        print("wrist_1_joint: ", self.jointSpeed[3])
        print("wrist_2_joint: ", self.jointSpeed[4])
        print("wrist_3_joint: ", self.jointSpeed[5])
        print()
        print("~ Sim Info ~")
        print("shoulder_pan_joint: ", self.an1, "\t",self.v1)
        print("shoulder_lift_joint: ", self.an2, "\t",self.v2)
        print("elbow_joint: ", self.an3, "\t",self.v3)
        print("wrist_1_joint: ", self.an4, "\t",self.v4)
        print("wrist_2_joint: ", self.an5, "\t",self.v5)
        print("wrist_3_joint: ", self.an6, "\t",self.v6)
        print()

        if self.J_inv is not None:
            print("~ Jacobian Matrix ~")
            print("Det(J): ", np.linalg.det(self.J))
            print('\n'.join(['\t'.join([str(round(cell,3)) for cell in row]) for row in self.J]))
        
        if self.tmpIndex is not None:
            print()
            print("~ Interpolation Info ~ ")
            print(self.tmpIndex, "/", self.maxIndex+1, " step.")
        
        if self.error is not None:
            print()
            print("~ Error ~")
            print('\t'.join([str(round(cell,4)) for cell in self.error]))
            print("P: " + str(self.P))
            print("I: " + str(self.I))
            print("D: " + str(self.D))
            if self.num_error is not None:
                print("Error (scalar): ", self.num_error)

    def callback(self,msg):

        #This works fine

        self.an3, self.an2, self.an1, self.an4, self.an5, self.an6 = msg.position
        self.v3, self.v2, self.v1, self.v4, self.v5, self.v6 = msg.velocity

        self.an = [self.an1, self.an2, self.an3, self.an4, self.an5, self.an6]
        self.v = [self.v1, self.v2, self.v3, self.v4, self.v5, self.v6]
    
    def goalcallback(self,msg):

        data = [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw]
        self.setGoal(data)

    def speedCallback(self,msg):

        speed = msg.data
        self.goalSpeed = speed

    def quaternionInterpolation(self, steps=10):

        #WIP - This function is the holy grail of this project
        #Idea - SLERP

        #Lord have mercy

        w1, x1, y1, z1 = get_quaternion_from_euler(self.pose6[3], self.pose6[4], self.pose6[5])
        w2, x2, y2, z2 = get_quaternion_from_euler(self.goal[3], self.goal[4], self.goal[5])

        quat_start = np.quaternion(w1, x1, y1, z1)
        quat_goal = np.quaternion(w2, x2, y2, z2)

        interp = quaternion.slerp(quat_start, quat_goal, 0, 1, np.arange(0,1,1/steps))
        
        interp = [np.array(euler_from_quaternion(elem.w, elem.x, elem.y, elem.z)) for elem in interp]

        #self.interpolations = interp

        #By now we have calculated all the relevant interpolations, so we need to add them to the goal.

        #If this does not work, just try again with different ideas

        #Now what?

        return interp

    def poseInterpolation(self, steps=10):

        #WIP - This function should just get us n equal segments for a trajectory in Cartesian-RPY space.
        #Assumptions:

        #Cartesian space is linear, so we can just draw a linear coordinate approach to this
        #We are using single quaternions

        initial_position = np.array(self.pose6[:3])
        final_position = np.array(self.goal[:3])

        vec = (final_position - initial_position)/steps

        translationPoints = [initial_position + vec*i for i in range(steps)]
        rotationPoints = self.quaternionInterpolation(steps)

        self.interpolations = [np.append(a,b) for a,b in zip(translationPoints, rotationPoints)]
        self.tmpIndex = 0
        self.maxIndex = steps-1
        self.tmpGoal = self.interpolations[0]

        pass

    def getRotError(self, eul):

        """Computes the rotation error between the current rotation and a specified one\n
        Returns a RPY vector which can be used as input for the Jacobian inverse."""

        w, x, y, z = get_quaternion_from_euler(eul[3],eul[4],eul[5])
        finalQt = np.quaternion(w, x, y, z)
        currentQt = self.qt

        errorqt = finalQt * currentQt.conjugate()
        trueEulError = np.array(euler_from_quaternion(errorqt.w, errorqt.x, errorqt.y, errorqt.z))

        return trueEulError

    def calcH(self,n, debug = False):
            
        H = np.array([[cos(self.an[n-1]), -sin(self.an[n-1])*cos(self.alpha[n-1]), sin(self.an[n-1])*sin(self.alpha[n-1]), self.a[n-1]*cos(self.an[n-1])],[sin(self.an[n-1]), cos(self.an[n-1])*cos(self.alpha[n-1]), -cos(self.an[n-1])*sin(self.alpha[n-1]), self.a[n-1]*sin(self.an[n-1])],[0, sin(self.alpha[n-1]), cos(self.alpha[n-1]), self.d[n-1]],[0, 0, 0, 1]])

        if debug:

            print("Joint number: " + str(n))

            print("DH Params")
            print("a: " + str(self.a[n-1]))
            print("alpha: " + str(self.alpha[n-1]))
            print("d: " + str(self.d[n-1]))
            print("theta: " + str(self.an[n-1]))
            print()
            print(H)
        
        return H

    def calcEveryH(self):

        if self.an1 is not None:
            matrixList = []
            for i in range(1,len(self.an)+1):
                matrixList.append(self.calcH(i)) 
            
            self.KinChain.update(matrixList)

    def messySolution(self):
        """This returns a 180 degree rotation matrix in the Z axis. THis fixes the robot's FK for Gazebo"""

        H = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])
        return H

    def getPose(self,H):

        #Y=atan2(H[1,0],H[0,0])
        #P=atan2(-H[2,0],sqrt(H[2,1]**2+H[2,2]**2))
        #R=atan2(H[2,1],H[2,2])

        P = atan2(-H[2,0],sqrt(H[2,1]**2+H[2,2]**2))
        R = atan2(H[2,1]/cos(P), H[2,2]/cos(P))
        Y = atan2(H[1,0]/cos(P), H[0,0]/cos(P))

        pose = [H[0,3], H[1,3], H[2,3], R, P, Y]
        return pose

    def FK(self):

        if self.KinChain.size != 0:

            FKMatrixList = self.KinChain.ForwardKinematics()

            self.pose1 = self.getPose(FKMatrixList[0])
            self.pose2 = self.getPose(FKMatrixList[1])
            self.pose3 = self.getPose(FKMatrixList[2])
            self.pose4 = self.getPose(FKMatrixList[3])
            self.pose5 = self.getPose(FKMatrixList[4])
            self.pose6 = self.getPose(FKMatrixList[5])

            H = FKMatrixList[5]

            #print(H)
            self.EErotation = H[0:3,0:3]
            self.EEtranslation = H[0:3,3].flatten()
            self.EEHomMat = H
            w, x, y, z = get_quaternion_from_euler(self.pose6[3],self.pose6[4],self.pose6[5])
            self.qt = np.quaternion(w,x,y,z)

        else:

            self.pose6 = [None]*6
    
    def calculateJ(self, inv=True):

        lmbda = 0.1

        #It works just fine

        #Calculate a Jacobian matrix for the current position of the arm
        #By this point you should have already calculated FK successfully
        #For each column:
        #First 3 elems = (R0_n * [0,0,1]) * (t_6 - t_n)
        #Last 3 elems = (R0_n * [0,0,1])
        #If inv, returns the inverse Jacobian matrix

        J_t = np.array([])
        current_H = self.KinChain.base

        for i in range(6):
            
            # Step 1: get the right transformation matrix

            current_z = current_H[0:3,2]
            current_T = current_H[0:3,3]
            rotation_ZAxis = current_z.flatten()

            first_half  = np.cross(rotation_ZAxis,self.EEtranslation - current_T)
            row = np.concatenate([first_half, rotation_ZAxis])

            if J_t.size != 0:
                J_t = np.vstack((J_t, row))
            else:
                J_t = np.append(J_t, row)
            
            current_H = current_H @ self.KinChain.homogeneousMatrixList[i]
            
        J = np.transpose(J_t)

        self.J = J

        if inv:
            if abs(np.linalg.det(J)) <= 0.05:

                self.J_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + lmbda**2 * np.eye(J.shape[0])))
            
            else:

                self.J_inv = np.linalg.inv(J)

        pass

    def calculateJointSpeed(self, desired_end_speed):
        if self.J_inv is not None:
            desired_end_speed = np.array(desired_end_speed).reshape((-1,1))
            Jinv = self.J_inv
            joint_speed = (Jinv @ desired_end_speed).flatten()
            self.jointSpeed = joint_speed
            return joint_speed

        else:
            return None
    
    def publishJointSpeed(self, jointspeed):
        
        msg = Float64MultiArray()
        #Use a controller to reach desired speed in a certain time i.e. 100 iterations
        #Achieved by a proportional controller - just P
        speed = [self.v[i] + ((jointspeed[i] - self.v[i])/self.K) for i in range(0,6)]

        #speed = [0,-0.1,0,0,0,0]
        self.jointSpeed = speed
        self.clampSpeed()   
        msg.data = self.jointSpeed
        self.velPub.publish(msg)
    
    def setGoal(self,goal,interp=False):
        self.goal = goal
        if interp:
            while True:
                self.calcEveryH()
                self.FK()
                if self.pose6 is not None:
                    break
            self.poseInterpolation()
        
        else:
            self.tmpGoal = goal
            self.maxIndex = 0
            self.tmpIndex = 0
    
    def getError(self,goal,scalar=False):

        error = np.array([(goal[i] - self.pose6[i]) for i in range(3)])

        w, x, y, z = get_quaternion_from_euler(goal[3],goal[4],goal[5])
        finalQt = np.quaternion(w, x, y, z)
        currentQt = self.qt

        errorqt = finalQt * currentQt.conjugate()
        trueEulError = np.array(euler_from_quaternion(errorqt.w, errorqt.x, errorqt.y, errorqt.z))

        error = np.append(error, trueEulError)
        
        if scalar:
            error = np.linalg.norm(error)
            self.num_error = error
        

        return error
    
    def reachPose(self):

        msg = Float64MultiArray()
        goal = self.tmpGoal

        if self.getError(goal, scalar=True) < 0.1:
            
            if self.tmpIndex <= self.maxIndex:
                self.tmpIndex += 1


            if self.tmpIndex > self.maxIndex:
                goal = self.goal
            
            else:
                
                self.tmpGoal = self.interpolations[self.tmpIndex]
                goal = self.tmpGoal

        
        #Use a similar controller to approach the desired coordinates for the end effector

        if all([(abs(self.goal[i] - self.pose6[i])< 0.001) for i in range(6)]):

            msg.data = [0,0,0,0,0,0]
            #print("Reached goal with precision > 0.01")

        else:

            #TODO: Be able to calculate dt after each and every iteration / done?
            #PID Control
            #Normalize the speed vector
            #This is possible since the Jacobian matrix is a linear approximation of end-effector kinematics for pose.

            #error = np.array([(goal[i] - self.pose6[i]) for i in range(6)])
            #self.error = error

            #This is not as accurate as I thought, so let's remake it

            error = np.array([(goal[i] - self.pose6[i]) for i in range(3)])

            w, x, y, z = get_quaternion_from_euler(goal[3],goal[4],goal[5])
            finalQt = np.quaternion(w, x, y, z)
            currentQt = self.qt

            errorqt = finalQt * currentQt.conjugate()
            trueEulError = np.array(euler_from_quaternion(errorqt.w, errorqt.x, errorqt.y, errorqt.z))

            error = np.append(error, trueEulError)
            self.error = error

            #Let's normalize linear and angular speed separatedly.
            #lin_speed = speed[:3]/np.linalg.norm(speed[:3])
            #ang_speed = speed[3:]/np.linalg.norm(speed[3:])
            #speed = np.append(lin_speed, ang_speed)

            #Calculate P error
            P = error * self.Kp

            #Calculate I error
            if self.sum_e is not None:
                self.sum_e += (error * self.dt)
                
            else:
                self.sum_e = error * self.dt
            
            I = self.sum_e * self.Ki


            #Calculate D error
            if self.prev_e is not None:
                D = ((error - self.prev_e) / self.dt) * self.Kd

            else:
                D = [0,0,0,0,0,0]
            
            self.prev_e = error
            self.P, self.I, self.D = P, I, D
            
            endspeed = (P+I+D).tolist()
            jointspeed = self.calculateJointSpeed(endspeed)
            self.publishJointSpeed(jointspeed)
            msg.data = jointspeed

        self.velPub.publish(msg)

    def reachSpeed(self):

        self.calculateJointSpeed(self.goalSpeed)
        self.clampSpeed()
        self.publishJointSpeed(self.jointSpeed)

    def clampSpeed(self):

        first = np.clip(np.array(self.jointSpeed[:2]), self.clamp_speed_1[0], self.clamp_speed_1[1])
        second = np.clip(np.array(self.jointSpeed[2:]), self.clamp_speed_2[0], self.clamp_speed_2[1])

        self.jointSpeed = np.append(first, second)

        final_poses = np.array([self.a[i] + self.jointSpeed[i]*self.dt for i in range(6)])
        valid = (final_poses>=self.clamp_angle[0]) & (final_poses<=self.clamp_angle[1])

        self.jointSpeed *= valid

    def onShutdown(self):
        msg = Float64MultiArray()
        msg.data = [0,0,0,0,0,0]
        self.velPub.publish(msg)

    def checkCollision(self,jointState):
        
        #WIP

        pass








def main():
    now = datetime.datetime.now()
    nowstr = now.strftime("%d_%m_%y_%H_%M_%S")
    logname = "log_"+nowstr+".txt"
    location = str(pathlib.Path(__file__).parent.resolve()) + "/logs/"
    logpath = location + logname
    rospy.init_node("CartesianController")
    robot = UR10e(logfile=logpath, sim=True)
    cnt = 0
    ref_rate = 1000
    save_rate = 10
    time1 = time.time()

    while not rospy.is_shutdown():
        
        if robot.an1 is not None:

            robot.calcEveryH()
            robot.FK()
            robot.calculateJ()

            if robot.goal is not None:
                robot.reachPose()
            
            elif robot.goalSpeed is not None:
                robot.reachSpeed()
            
            #joint_speed = robot.calculateJointSpeed(speed)
            #robot.publishJointSpeed(joint_speed)
            #print("Moving")

            if cnt % save_rate == 0:
                robot.logWrite()
        
        cnt += 1
        if cnt % ref_rate == 0:
            os.system("clear")
            robot.printPose((time.time()-time1)/ref_rate)
            #robot.printDebug(flag="Pose")
            time1 = time.time()
            pass

    


if __name__ == "__main__":
    main()