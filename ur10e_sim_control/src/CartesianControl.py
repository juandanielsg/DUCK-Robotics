#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from immortals_messages.msg import EulerPose, Path, EulerPoseArray, EulerPoseAndFlags, PathRequest, IKReply, IKRequest
import numpy as np
import math
import scipy
from math import sin, cos, atan2, sqrt
import os
import time
import datetime
import quaternion
import pathlib

from ur10e_sim_control.Utility import get_quaternion_from_euler, euler_from_quaternion, KinematicChain, build_se3_transform, list2Pose, pose2Array, getErrorVectors, euler_to_so3, getVelocityRCM, calculate_velocity, euler_from_vectors, getH_bc, getErrorVec

class UR():

    __slots__ = {'K', 'd', 'a', 'alpha', 'goalSub', 'poseSub', 'velSub', 'velPub', 'posePub', 'posesPub', 'pathPub', 
                 'planRequester', 'planSub', 'an1', 'an2', 'an3', 'an4', 'an5', 'an6', 'an7', 'v3', 'v2', 'v1', 'v4', 'v5', 'v6',
                 'an', 'v', 'clamp_speed_1', 'clamp_speed_2', 'clamp_speed_3', 'clamp_speed_4', 'clamp_angle', 'KinChain', 'tool', 'EEHomMat', 'EErotation', 'EEtranslation',
                 'pose1', 'pose2', 'pose3', 'pose4', 'pose5', 'pose6', 'toolpose', 'J', 'J_inv', 'jointSpeed', 'Kp', 'Ki', 'Kd', 'dt', 'P', 'I', 'D',
                'error', 'prev_e', 'sum_e', 'logfilename', 'log', 'qt', 'interpolations', 'num_error', 'goal', 'goalSpeed', 'goalqt', 'tmpGoal', 'tmpIndex', 'maxIndex',
                'receivedGoal', 'planning', 'solvingIK', 'receivedConstraint', 'useConstraint', 'controlTool', #Flags
                 'goal_angles', 'IKPub', 'IKSub', 'controller', 'controllerFunc', 'previousPose', 'currentEstimatePose', 'currentEToolPose',
                 'constraint', 'constraintVec', 'constraintSub', 'constraintPub', 'baseVec','currentCartesianSpeed','rcmpose'
                }

    def __init__(self, logfile=None, sim=True, tool='none', controller='cartP'):
        
        # Constant
        self.baseVec = np.array([0,0,1])

        # K value
        self.K = 0.01
        
        #Denavit-Hartenberg parameters - Not modifiable.
        self.d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]
        self.a = [0, -0.612, -0.5723, 0, 0, 0]
        self.alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]

        #Current link positions
        self.an3, self.an2, self.an1, self.an4, self.an5, self.an6, self.an7 = None, None, None, None, None, None, None
        self.v3, self.v2, self.v1, self.v4, self.v5, self.v6 = None, None, None, None, None, None

        self.an = [None, None, None, None, None, None, None]
        self.v = []

        #Maximum joint positions and speed - WIP. Add

        self.clamp_angle = [-np.pi*2, np.pi*2]

        self.clamp_speed_1 = np.array([-0.05, -0.05, -0.05, -0.05, -0.05, -0.05])
        self.clamp_speed_2 = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])

        self.clamp_speed_3 = np.array([-0.1, -0.1, -0.1, -0.1, -0.1, -0.1])
        self.clamp_speed_4 = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])


        # **NOVELTY** Added Kinematic chain to the system

        self.KinChain = KinematicChain()
        if sim:
            self.KinChain.setBase(self.messySolution())
        else:
            self.KinChain.setBase(np.eye(4))
        

        #Tool attachments

        self.tool = True if tool else False

        if tool:
            
            self.attachTool(tool)
        

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
        self.toolpose = None
        self.rcmpose = None

        #Jacobian
        self.J = np.array([[1,0],[1,0]])
        self.J_inv = None

        #Current Speed
        self.jointSpeed = [None]*6

        #PID Params - Here is something that I want to fix.

        #Before -> 1 less 0 on each one. Perhaps even 2
        
        #HERE: Change your PID values

        #self.Kp = 0.0001
        #self.Ki = 0.00001
        #self.Kd = 0.000003
        
        #HERE: Change your PID values

        self.Kp = 0.9
        self.Ki = 0.0
        self.Kd = 0.0

        self.dt = 0.001

        self.P = None
        self.I = None
        self.D = None

        self.error = None
        self.prev_e = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.sum_e = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

        self.previousPose = None
        self.currentEstimatePose = None
        self.currentEToolPose = None
        self.currentCartesianSpeed = None

        #Generate a new logfile to collect data - WIP
        if logfile is not None:
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
        self.tmpGoal = None
        self.tmpIndex = None
        self.maxIndex = None

        #Joint-Space goals and data
        self.goal_angles = None

        #Flags
        self.receivedGoal = False
        self.planning = False
        self.receivedConstraint = False
        self.solvingIK = False
        self.useConstraint = False
        self.controlTool = False

        #Setting up the controller to use
        self.controller = controller
        self.controllerFunc = None
        self.setController(self.controller)

        #Setting up the constraint
        self.constraint = None
        self.constraintVec = None

        #ROS publishers/subscribers
        self.goalSub = rospy.Subscriber("/goal_pose", Twist, self.goalcallback, queue_size=1)
        self.poseSub = rospy.Subscriber("/joint_states", JointState, self.callback)
        self.velSub = rospy.Subscriber("/goal_speed", Twist, self.speedCallback, queue_size=1)
        self.velPub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        self.posePub = rospy.Publisher("/current_pose", Pose, queue_size=1)
        self.posesPub = rospy.Publisher("/current_poses", EulerPoseArray, queue_size=1)
        self.pathPub = rospy.Publisher("/path_plan",Path,latch=True, queue_size=1)
        self.constraintSub = rospy.Subscriber("/constraint", Pose, queue_size=1, callback=self.constraintCallback)

        #Planning ROS agents
        self.planRequester = rospy.Publisher("/path_requests", PathRequest, latch=True, queue_size=1)
        self.planSub = rospy.Subscriber("/path_responses", Path, self.planCallback, queue_size=1)

        #IK ROS agents
        self.IKPub = rospy.Publisher("/IK_requests", IKRequest, queue_size=1)
        self.IKSub = rospy.Subscriber("/IK_replies", IKReply, self.IKCallback, queue_size=1)

        # Do something oshutdownn 
        rospy.on_shutdown(self.onShutdown)

    def setGains(self, p, i, d):

        self.Kp = p
        self.Ki = i
        self.Kd = d

    def attachTool(self, toolname):
        
        toolpath = str(pathlib.Path(__file__).parent.resolve()) + "/tools/" + toolname + ".tool"

        try:
            
            with open(toolpath, 'r') as file:

                data = [line.strip("\n") for line in file.readlines()]
                data = [line for line in data if line[0] != "#"]
                data = [line.split(" ") for line in data]
                data = [float(datum) for datum in data[0]]
                #print(data)
                theta, a, d, alpha = data
                self.a.append(a)
                self.d.append(d)
                self.alpha.append(alpha)
                self.an7 = theta
                self.an[6] = theta

                mat = self.calcH(7)
                self.KinChain.setTool(mat)

        except:

            print("ERROR: tool file not found.")
    
    def setController(self, key) -> None:

        funcs = {"cartP": self.cartesianPoseControl, "cartV": self.cartesianVelocityControl, "jointP": self.jointSpacePoseControl, "jointV": self.jointSpaceVelocityControl, "pureCartV": self.cartesianVelocityPureControl}
        self.controllerFunc = funcs[key]
        
    def setTime(self, time):
        self.dt = time

    def initFile(self) -> None:
        """Creates a new log file in the logs/ directory to store data during the execution of the program. It will override any log files with the same name, but this should not happen.
        """

        self.log = open(self.logfilename, "w")

    def array2String(self, pose) -> str:
        """Converts a pose (6 value array) into a string. This is a method called by logWrite() only, and it has no other use.

        Args:
            pose (np.array): a 6 value array in the form of [x,y,z,roll,pitch,yaw] which defines space poses in the Cartesian/Euler space.

        Returns:
            string: a string in the format x/y/z/roll/pitch/yaw with a line jump at the end.
        """
        
        return "/".join([str(coord) for coord in pose]) + "\t" if pose is not None else ''
    
    def logWrite(self) -> None:
        
        data = self.array2String(self.toolpose) + self.array2String(self.pose6) + self.array2String(self.P) + self.array2String(self.I) + self.array2String(self.D) + self.array2String(self.jointSpeed) + str(self.dt) + '\n'

        self.log.write(data)

    def printPose(self, time) -> None:
        print("~ Average iteration time ~")
        print(str(time) + " s")
        self.dt = time
        print()
        if self.pose6 is not None:
            
            print("~ POSE ~")
            print("X: ", self.toolpose[0])
            print("Y: ", self.toolpose[1])
            print("Z: ", self.toolpose[2])
            print("R: ", self.toolpose[3])
            print("P: ", self.toolpose[4])
            print("Y: ", self.toolpose[5])

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

        print("~ Flags ~")
        print("Goal received: ", self.receivedGoal)
        print("Planning path: ", self.planning)
        print("Constraint received: ", self.receivedConstraint)
        print("Solving IK: ", self.solvingIK)
        print("Using constraint for movement: ", self.useConstraint)
        print()

        if self.J_inv is not None:
            print("~ Jacobian Matrix ~")
            print("Det(J): ", np.linalg.det(self.J))
            print('\n'.join(['\t'.join([str(round(cell,3)) for cell in row]) for row in self.J]))

        if self.goalSpeed is not None:
            print("~ Current desired speed ~")
            print(self.goalSpeed)
            print(self.jointSpeed)
        
        if self.tmpIndex is not None:
            print()
            print("~ Interpolation Info ~ ")
            print(self.tmpIndex, "/", self.maxIndex+1, " step.")
            print(self.tmpGoal)
            print(self.goal)
        
        if self.error is not None:
            print()
            print("~ Error ~")
            print('\t'.join([str(round(cell,4)) for cell in self.error]))
            print("P: " + str(self.P))
            print("I: " + str(self.I))
            print("D: " + str(self.D))
            if self.num_error is not None:
                print("Error (scalar): ", self.num_error)
            
#            print("Current EE Pose: ", *np.round(self.pose6,3))
#            print("Expected pose: ", *np.round(self.currentEstimatePose,3))

#            print("Current tooltip Pose: ", *np.round(self.toolpose,3))
#            print("Expected tootip Pose: ", *np.round(self.currentEToolPose,3))
    
    def printPoses(self):

        print(self.pose1)
        print(self.pose2)
        print(self.pose3)
        print(self.pose4)
        print(self.pose5)
        print(self.pose6)
        print(self.toolpose)
        print()

    def callback(self,msg):

        #This works fine

        self.an3, self.an2, self.an1, self.an4, self.an5, self.an6 = msg.position
        self.v3, self.v2, self.v1, self.v4, self.v5, self.v6 = msg.velocity

        self.an = [self.an1, self.an2, self.an3, self.an4, self.an5, self.an6, self.an7]
        self.v = [self.v1, self.v2, self.v3, self.v4, self.v5, self.v6]
    
    def goalcallback(self,msg):

        data = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        self.receivedGoal = True
        self.useConstraint = msg.constraint

        if self.controller == "cartP" or self.controller == "jointP":
            if msg.constraint:
                self.setGoal(data, plan=False, interp=False)
            
            else:
                self.setGoal(data, plan=msg.plan, interp=True)
        
        else:
            self.goalSpeed = msg.data

    def speedCallback(self,msg):

        speed = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        self.goalSpeed = np.array(speed)
        self.useConstraint = True
        self.currentEstimatePose, self.currentEToolPose = None, None
        self.sum_e = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.prev_e = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    
    def planCallback(self, msg):

        if msg.success:

            self.interpolations = [pose2Array(pose) for pose in msg.trajarray]
            self.tmpIndex = 0
            self.maxIndex = len(self.interpolations)-1
            self.tmpGoal = self.interpolations[0]

            msg = Path()
            msg.trajarray = [list2Pose(self.toolpose)] + [list2Pose(pose) for pose in self.interpolations] + [list2Pose(self.goal)]
            self.pathPub.publish(msg)
            self.planning = False

    def IKCallback(self, msg):

        self.goal_angles = np.array(msg.angles)
        self.solvingIK = False

    def constraintCallback(self, msg):

        self.constraint = np.array([msg.position.x, msg.position.y, msg.position.z, 0.0, 0.0, 0.0])
        self.receivedConstraint = True
        mat = euler_to_so3(self.constraint[3:])
        self.constraintVec = mat @ self.baseVec

    def quaternionInterpolation(self, steps=10):

        #WIP - This function is the holy grail of this project 
        #This used to be the holy grail, now it is just a small piece.
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

        P = atan2(-H[2,0],sqrt(H[2,1]**2+H[2,2]**2))
        R = atan2(H[2,1]/cos(P), H[2,2]/cos(P))
        Y = atan2(H[1,0]/cos(P), H[0,0]/cos(P))

        pose = np.array([H[0,3], H[1,3], H[2,3], R, P, Y])
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
            self.toolpose = self.getPose(FKMatrixList[6])

            H = FKMatrixList[5]

            if self.controlTool:

                H = FKMatrixList[6]
                
            self.EErotation = H[0:3,0:3]
            self.EEtranslation = H[0:3,3].flatten()
            self.EEHomMat = H

            if self.useConstraint and False:

                self.FK_constraint()

            w, x, y, z = get_quaternion_from_euler(self.toolpose[3],self.toolpose[4],self.toolpose[5])
            
            self.qt = np.quaternion(w,x,y,z)

            msg = Pose()

            msg.position.x = self.toolpose[0]
            msg.position.y = self.toolpose[1]
            msg.position.z = self.toolpose[2]
            msg.orientation.w = w
            msg.orientation.x = x
            msg.orientation.y = y 
            msg.orientation.z = z


            self.posePub.publish(msg)

            msg2 = EulerPoseArray()
            msg2.element_poses = [list2Pose(self.pose1), list2Pose(self.pose2), list2Pose(self.pose3), list2Pose(self.pose4), list2Pose(self.pose5), list2Pose(self.pose6), msg]
            self.posesPub.publish(msg2)

            

        else:

            self.pose6 = [None]*6
    
    def FK_constraint(self):

        """Computes forward kinematics considering that we add a virtual link between the end effector and the constraint.
        """

        #3 Principles

        #1. The Jacobian would be computed from the rcm pose rather than our own.
        #2. We assume the rcm pose to be aligned to the ee pose (Technically right). The rcm for us is a point in space.
        
        #Therefore...

        #NONONONONONONO, will have to check this one later.

        H = getH_bc(self.pose6, self.constraint, aligned=True)
        self.EEHomMat = self.EEHomMat @ H
        self.EEtranslation = self.EEHomMat[0:3,3].flatten()
        self.rcmpose = self.getPose(self.EEHomMat)

        #Note: after enabling this, the new Jacobian will be a RCM jacobian (theoretically???)

        pass

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

                #self.J_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + lmbda**2 * np.eye(J.shape[0]))) - Damped least squares: use at your own risk
                self.J_inv = np.linalg.pinv(J)
            
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
        self.jointSpeed = jointspeed
        #self.clampSpeed()   
        msg.data = self.jointSpeed
        self.velPub.publish(msg)
    
    def setGoal(self,goal,plan=False,interp=False):

        self.sum_e = None
        self.prev_e = None

        msg = Path()

        if self.controller == "cartP":

            self.goal = goal
            msg = Path()

            if plan:

                while self.toolpose is None:
                    self.calcEveryH()
                    self.FK()
                
                self.plan(goal)

            else:

                self.planning = False

                if interp:
                    while True:
                        self.calcEveryH()
                        self.FK()
                        if self.toolpose is not None:
                            break
                    self.poseInterpolation()
                    msg.trajarray = [list2Pose(self.toolpose)] + [list2Pose(pose) for pose in self.interpolations] + [list2Pose(self.goal)]
                    self.pathPub.publish(msg)
                        
                
                else:
                    self.tmpGoal = goal
                    self.maxIndex = 0
                    self.tmpIndex = 0

                    msg.trajarray = [list2Pose(self.tmpGoal)]
                    self.pathPub.publish(msg)
                    #print(msg.trajarray)
        
        elif self.controller=="jointP":

            while self.an1 is None:
                pass
            
            while self.toolpose is None:
                self.calcEveryH()
                self.FK()

            msg.trajarray = [list2Pose(self.toolpose)] + [list2Pose(goal)]
            self.pathPub.publish(msg)
            self.solveIK(goal)
            
    def solveIK(self, goal):

        self.maxIndex = 0
        self.tmpIndex = 0
        self.solvingIK = True
        self.goal_angles = np.array(self.an.copy())

        msg = IKRequest()
        msg.start_angles = self.an
        msg.end = list2Pose(goal)
        msg.maxtime = 10.0
        self.IKPub.publish(msg)

    def plan(self, goal):

        self.maxIndex = 0
        self.tmpIndex = 0
        self.planning = True
        self.tmpGoal = self.toolpose.copy()

        msg = PathRequest()
        msg.maxtime = 100.0
        msg.start = list2Pose(self.toolpose)
        msg.end = list2Pose(goal)
        msg.start_angles = self.an

        self.planRequester.publish(msg)
    
    def getError(self,goal,scalar=False):

        if self.controlTool:
            
            """goalMat = build_se3_transform(goal)
            #print(goalMat)
            #print(self.KinChain.invtool)
            goalMat = goalMat @ self.KinChain.invtool

            #If constraint, goal should be measured on the tool tip, not on the ee
            goal = self.getPose(goalMat)"""
            
            error = np.array([(goal[i] - self.toolpose[i]) for i in range(3)])

        else:

            error = np.array([(goal[i] - self.pose6[i]) for i in range(3)])

        w, x, y, z = get_quaternion_from_euler(goal[3],goal[4],goal[5])
        finalQt = np.quaternion(w, x, y, z)
        currentQt = self.qt

        errorqt = finalQt * currentQt.conjugate()
        trueEulError = np.array(euler_from_quaternion(errorqt.w, errorqt.x, errorqt.y, errorqt.z))

        error_ = np.append(error, trueEulError)
        
        if scalar:
            error = np.linalg.norm(error) + np.linalg.norm(trueEulError)
            self.num_error = error
            return error
        

        return error_
    
    def reachPose(self):

        msg = Float64MultiArray()
        goal = self.tmpGoal

        #print(self.tmpGoal)

        #print(goal)

        if self.getError(goal, scalar=True) < 0.05 and not self.planning:
            
            self.sum_e = None
            if self.tmpIndex <= self.maxIndex:
                self.tmpIndex += 1


            if self.tmpIndex > self.maxIndex:
                goal = self.goal
                self.tmpGoal = self.goal
            
            else:
                
                self.tmpGoal = self.interpolations[self.tmpIndex]
                goal = self.tmpGoal

        
        #Use a similar controller to approach the desired coordinates for the end effector

        #Now we have to estimate the error on the EE, not on the tool

        if self.useConstraint:
            error_range = 3
        
        else:
            error_range = 6

        if all([(abs(self.goal[i] - self.toolpose[i])< 0.001) for i in range(error_range)]) and not self.useConstraint:

            msg.data = [0,0,0,0,0,0]
            self.velPub.publish(msg)
            #print("Reached goal with precision > 0.01")

        else:

            #IDEA: if using constraint, we should use v_tool as seen from ee: error(ee, goal) - error (ee, toolnow) for linear

            #Given T = tool frame

            

            #TODO: Be able to calculate dt after each and every iteration / done?
            #PID Control
            #Normalize the speed vector
            #This is possible since the Jacobian matrix is a linear approximation of end-effector kinematics for pose.

            #error = np.array([(goal[i] - self.pose6[i]) for i in range(6)])
            #self.error = error

            #This is not as accurate as I thought, so let's remake it

            #If I have H_bee and H_btool and H_brcm
            #And I want H_ee_tool and H_ee_rcm and H_rcm_tool

            if self.receivedConstraint and self.useConstraint:
                error = self.getError(goal,scalar=False)[:3]
                error = getVelocityRCM(self.constraint, self.toolpose, self.pose6, error)
            
            else:
                error = self.getError(goal,scalar=False)

            self.error = error

            #Let's normalize linear and angular speed separatedly.
            #lin_speed = speed[:3]/np.linalg.norm(speed[:3])
            #ang_speed = speed[3:]/np.linalg.norm(speed[3:])
            #speed = np.append(lin_speed, ang_speed)

            #Calculate P error
            #P = error

            #Assume Kp = 1 in this case. Otherwise, uncomment:
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
            if self.receivedConstraint and self.useConstraint:
                endspeed = getVelocityRCM(self.constraint, self.toolpose, self.pose6, endspeed)
            jointspeed = self.calculateJointSpeed(endspeed)
            self.publishJointSpeed(jointspeed)

    def reachPose_rcm(self):

        #The same but for RCM constraints

        pass

    def reachSpeed(self):

        #IT WORKS NOW :D

        #Now add the vector for the RCM, if you dare


        desired_end_speed = np.array(self.goalSpeed)
        
        eTE = self.KinChain.invtool
        
        #vel = desired_end_speed[:3] - np.cross(np.array(desired_end_speed[3:]), eET[0:3,3])

        #V_end_effector = V_tool + omega_tool × (R * M)

        #Option 2: turn goalSpeed into goalSpeed from ee perspective, then run all calculations
        #and turn the result back to goal speed. This could solve my problems or prove my system to be wrong. 

        #True problem. We are not keeping in mind the current desired pose of the TOOL when running all these calculations.
        #Therefore, they are all wrong.

        if self.currentEstimatePose is None:

            self.currentEstimatePose = self.pose6
            self.currentEToolPose = self.toolpose

        #vel = calculate_velocity(self.currentEToolPose[:3], self.currentEstimatePose[:3],self.goalSpeed[:3],self.goalSpeed[3:])

        #The rotation speed is inverse in the tool tip to the one on the end effector.
        #np.linalg.norm(eBT[0:3,3]) This was dividing -1 until I realized that it made no sense whatsoever.
        #desired_end_speed = np.append(vel, np.array([desired_end_speed[3:]]))
        #print(desired_end_speed)

        #self.desired_end_speed = getVelocityRCM(self.constraint, self.toolpose, self.pose6, self.goalSpeed)
    
        #My vector metric is the difference in pose between the current pose of the ee and the desired one.
        #I should be using a different metric perhaps.

        #Scrapped all previous effort as it was highly inefficient. Now lets see how can I get the desired end speed

        #How can the error be applied. IE: add the error to the current desired ee speed.

        #Formula: given error as a velocity vector in the form of (desiredpose-pose)/dt

        #P = Velocity
        #I = Distance
        #D = Acceleration


        _, linear, angular, _, _ = getErrorVectors(np.array(self.pose6),np.array(self.currentEstimatePose))
        error = np.append(linear, angular)/self.dt
        self.error = error

        if self.sum_e is not None:
            self.sum_e += (error * self.dt)
            
        else:
            self.sum_e = error * self.dt
        
        if self.prev_e is None:
            self.prev_e = self.error
        
        P = error * self.Kp
        I = self.sum_e * self.Ki
        D = (((error - self.prev_e)/self.dt) * self.Kd)

        self.P, self.I, self.D = P, I, D

        
        self.currentEToolPose = self.currentEToolPose + self.goalSpeed * self.dt
        #self.currentEstimatePose = self.currentEstimatePose + desired_end_speed * self.dt
        self.currentEstimatePose = self.getPose(build_se3_transform(self.currentEToolPose) @ eTE)

        #This fixes the position of the new point to the correct point (0.5m apart in this case)
        position_dif = self.currentEToolPose[:3] - self.currentEstimatePose[:3]
        normalized_vector = position_dif / np.linalg.norm(position_dif)
        self.currentEstimatePose = self.currentEToolPose - 0.5 * np.append(normalized_vector, np.zeros(3))

        desired_end_speed = P + I + D

        self.prev_e = error

        self.calculateJointSpeed(desired_end_speed)
        self.clampSpeed()
        self.publishJointSpeed(self.jointSpeed)

    def speedPIDControl(self):

        desired_speed = self.goalSpeed

        self.calculateJointSpeed(desired_speed)

        error = np.array(self.jointSpeed) - np.array(self.v)

        self.error = error

        if np.all(self.sum_e==0.0):
            self.sum_e += (error * self.dt)
            
        else:
            self.sum_e = error * self.dt
        
        if np.all(self.prev_e==0.0):
            self.prev_e = self.error
        
        P = error * self.Kp
        I = self.sum_e * self.Ki
        D = (((error - self.prev_e)/self.dt) * self.Kd)

        self.P, self.I, self.D = P, I, D

        self.publishJointSpeed(np.array(self.v) + (P + I + D))

    def reachRCMSpeed(self):

        """Does the same but with RCM. Requires the RCM to be in the tool.
        """

        #For the RCM, we can control position, not orientation. Therefore we will have the tooltip be oriented by a calculation.

        #Principle: given that we have a small enough time interval dt, we can approximate any trajectory as a series of linear speeds.
        
        if self.currentEstimatePose is None:
            self.previousPose = self.pose6
            self.currentEstimatePose = self.pose6
            self.currentEToolPose = self.toolpose

        error = getErrorVec(np.array(self.pose6),np.array(self.currentEstimatePose)) / self.dt
        
        self.error = error

        #desired_end_speed = [(self.goalSpeed[i]*self.K + (1-self.K) * (self.pose6[i]-self.previousPose[i])/self.dt) for i in range(0,6)]
        desired_end_speed = self.goalSpeed

        if self.sum_e is not None:
            self.sum_e += (error * self.dt)
            
        else:
            self.sum_e = error * self.dt
        
        if self.prev_e is None:
            self.prev_e = self.error
        
        P = error * self.Kp
        I = self.sum_e * self.Ki
        D = (((error - self.prev_e)/self.dt) * self.Kd)

        self.P, self.I, self.D = P, I, D

        #Let's get creative with this.

        self.currentEToolPose = self.currentEToolPose + self.goalSpeed * self.dt
        #self.currentEstimatePose = self.currentEstimatePose + desired_end_speed * self.dt

        #This fixes the position of the new point to the correct point (0.5m apart in this case)
        position_dif = self.currentEToolPose[:3] - self.constraint[:3]
        self.currentEToolPose = np.append(self.currentEToolPose[:3], euler_from_vectors(self.baseVec, position_dif))
        #print(self.currentEToolPose)
        normalized_vector = position_dif / np.linalg.norm(position_dif)
        self.currentEstimatePose = self.currentEToolPose - 0.5 * np.append(normalized_vector, np.zeros(3))

        desired_end_speed = P + I + D

        self.prev_e = error

        self.calculateJointSpeed(desired_end_speed)
        #self.clampSpeed()
        self.publishJointSpeed(self.jointSpeed)

    def reachRCMSpeed_method2(self):

        """Does the same but with RCM. Requires the RCM to be in the tool.
        """

        #For the RCM, we can control position, not orientation. Therefore we will have the tooltip be oriented by a calculation.

        #Now we want to control rotation at the RCM to control the next point.

        #Principle: We can control the tooltip with 2 rotations in the RCM and a translation along its own axis
        
        if self.currentEstimatePose is None:
            self.previousPose = self.rcmpose
            self.currentEstimatePose = self.rcmpose
            self.currentEToolPose = self.toolpose

        error = getErrorVec(np.array(self.rcmpose),np.array(self.currentEstimatePose)) / self.dt
        
        self.error = error

        #desired_end_speed = [(self.goalSpeed[i]*self.K + (1-self.K) * (self.pose6[i]-self.previousPose[i])/self.dt) for i in range(0,6)]
        desired_end_speed = self.goalSpeed

        if self.sum_e is not None:
            self.sum_e += (error * self.dt)
            
        else:
            self.sum_e = error * self.dt
        
        if self.prev_e is None:
            self.prev_e = self.error
        
        P = error * self.Kp
        I = self.sum_e * self.Ki
        D = (((error - self.prev_e)/self.dt) * self.Kd)

        self.P, self.I, self.D = P, I, D

        #Let's get creative with this.

        self.currentEToolPose = self.currentEToolPose + self.goalSpeed * self.dt
        #self.currentEstimatePose = self.currentEstimatePose + desired_end_speed * self.dt

        #This fixes the position of the new point to the correct point (0.5m apart in this case)
        position_dif = self.currentEToolPose[:3] - self.constraint[:3]
        self.currentEToolPose = np.append(self.currentEToolPose[:3], euler_from_vectors(self.baseVec, position_dif))
        #print(self.currentEToolPose)
        normalized_vector = position_dif / np.linalg.norm(position_dif)
        self.currentEstimatePose = self.currentEToolPose - 0.5 * np.append(normalized_vector, np.zeros(3))

        desired_end_speed = P + I + D

        self.prev_e = error

        self.calculateJointSpeed(desired_end_speed)
        #self.clampSpeed()
        self.publishJointSpeed(self.jointSpeed)

    def clampSpeed(self):

        if self.controller == "cartV":
            pass
        
        else:
            self.jointSpeed = np.clip(self.jointSpeed[:6], self.clamp_speed_3, self.clamp_speed_4)

        final_poses = np.array([self.a[i] + self.jointSpeed[i]*self.dt for i in range(6)])
        valid = (final_poses>=self.clamp_angle[0]) & (final_poses<=self.clamp_angle[1])

        self.jointSpeed *= valid

    def onShutdown(self):
        msg = Float64MultiArray()
        msg.data = [0,0,0,0,0,0]
        self.velPub.publish(msg)

    def cartesianPoseControl(self):

        """Loop for cartesian pose control of the end effector"""

        self.calcEveryH()
        self.FK()
        self.calculateJ()

        if self.tmpGoal is not None:
            self.reachPose()
            self.logWrite()

    def cartesianVelocityControl(self):

        """Loop for cartesian speed control of the end effector"""

        self.calcEveryH()
        self.FK()
        self.calculateJ()

        if self.goalSpeed is not None:
            if self.useConstraint:
                self.reachRCMSpeed()
            
            else:
                self.reachSpeed()
            
            self.logWrite()

    def cartesianVelocityPureControl(self):

        self.calcEveryH()
        self.FK()
        self.calculateJ()

        if self.goalSpeed is not None:
            
            self.speedPIDControl()
            self.logWrite()

    def jointSpacePoseControl(self):

        if self.goal_angles is not None:

            msg = Float64MultiArray()
            #STEP 1: Figure out end angles.

            #HERE: Implement an IK solver. This cannot be that hard, right?

            #STEP 2: Get error as end_angles - current_angles
            error = self.goal_angles[:6] - np.array(self.an[:6])
            self.error = error

            if all(error < 0.001):

                msg.data = [0,0,0,0,0,0]
                self.velPub.publish(msg)
                #print("Reached goal with precision > 0.01")

            else:
                #STEP 3: Use PID for said error

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

                self.jointSpeed = (P+I+D).tolist()
                #self.clampSpeed()

                #STEP 4: Publish the joint speeds
                self.publishJointSpeed(self.jointSpeed)
                self.logWrite()

    def jointSpaceVelocityControl(self):

        pass

def main():
    now = datetime.datetime.now()
    nowstr = now.strftime("%d_%m_%y_%H_%M_%S")
    logname = "log_"+nowstr+".txt"
    location = str(pathlib.Path(__file__).parent.resolve()) + "/logs/"
    logpath = location + logname
    rospy.init_node("CartesianController")

    controller = rospy.get_param('controller_type')
    tool = rospy.get_param('tool')

    robot = UR(logfile=logpath, sim=True, tool=tool, controller=controller)
    cnt = 0
    ref_rate = 100
    time1 = time.time()

    while not rospy.is_shutdown():
        
        if robot.an1 is not None:

            robot.controllerFunc()
            
        cnt += 1
        
        if cnt % ref_rate == 0:
            os.system("clear")
            robot.printPose((time.time()-time1)/ref_rate)
            time1 = time.time()
            pass

if __name__ == "__main__":
    main()