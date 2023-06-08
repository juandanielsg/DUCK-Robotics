
#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from immortals_messages.msg import Pose, PoseArray, Reply, Path, IKRequest, IKReply
import numpy as np
import math
from math import sin, cos, atan2, sqrt
import quaternion
import pathlib
import quaternion
import time

from ur10e_sim_control.Utility import KinematicChain, get_quaternion_from_euler, euler_from_quaternion, list2Pose, build_se3_transform, bcolors, pose2Array

class ClosedLoopUR10e():
        
    """
    Purely numerical representation of a UR10e robot. This is used for path or trajectory planning.
    """

    __slots__ = {'K', 'd', 'a', 'alpha', 'an1', 'an2', 'an3', 'an4', 'an5', 'an6', 'an7', 'v3', 'v2', 'v1', 'v4', 'v5', 'v6',
                 'an', 'v', 'clamp_speed_1', 'clamp_speed_2', 'clamp_angle', 'KinChain', 'tool', 'EEHomMat', 'EErotation', 'EEtranslation',
                 'pose1', 'pose2', 'pose3', 'pose4', 'pose5', 'pose6', 'toolpose', 'J', 'J_inv', 'jointSpeed', 'Kp', 'Ki', 'Kd', 'dt', 'P', 'I', 'D',
                'error', 'prev_e', 'sum_e', 'logfilename', 'log', 'qt', 'interpolations', 'num_error', 'goal', 'goalSpeed', 'goalqt', 'tmpGoal', 'tmpIndex', 'maxIndex',
                'answerSubscriber', 'posePublisher', 'collided', 'request_id', 'manipulability', 'manipulability_thresh'
                #Flags
                }

    def __init__(self, sub_topic, pub_topic, sim=True, tool='none', manipulability_thresh = 0.01):

        #ROS publishers/subscribers
        self.answerSubscriber = rospy.Subscriber(sub_topic, Reply, self.checkerCallback, queue_size=10)
        self.posePublisher = rospy.Publisher(pub_topic, PoseArray, queue_size=10)

        # Communication purposes
        self.collided = False
        self.request_id = 1
        self.manipulability = 2 #This is a placeholder and impossible value

        self.manipulability_thresh = manipulability_thresh

        # K value
        self.K = 1
        
        #Denavit-Hartenberg parameters.
        self.d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
        self.a = [0, -0.6127, -0.57155, 0, 0, 0]
        self.alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]

        #Current link positions.
        self.an3, self.an2, self.an1, self.an4, self.an5, self.an6, self.an7 = None, None, None, None, None, None, None
        self.v3, self.v2, self.v1, self.v4, self.v5, self.v6 = 0, 0, 0, 0, 0, 0

        self.an = [0]*7
        self.v = [0]*6

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
        

        #Tool attachments

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

        #Jacobian
        self.J = np.array([[1,0],[1,0]])
        self.J_inv = None

        #Current Speed
        self.jointSpeed = [None]*6

        #PID Params
        self.Kp = 1
        self.Ki = 0
        self.Kd = 0

        self.dt = 0.1

        self.P = None
        self.I = None
        self.D = None

        self.error = None
        self.prev_e = 0
        self.sum_e = 0

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

    def reset(self):

        self.P = None
        self.I = None
        self.D = None

        self.error = None
        self.prev_e = 0
        self.sum_e = 0

        self.v = [0,0,0,0,0,0]

    def attachTool(self, toolname):
        
        toolpath = str(pathlib.Path(__file__).parent.parent.resolve()) + "/tools/" + toolname + ".tool"

        try:
            
            with open(toolpath, 'r') as file:

                data = [line.strip("\n") for line in file.readlines()]
                data = [line for line in data if line[0] != "#"]
                data = [line.split(" ") for line in data]
                data = [float(datum) for datum in data[0]]

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
    
    def setAngles(self, angles):

        self.an1, self.an2, self.an3, self.an4, self.an5, self.an6, self.an7 = angles
        self.an = angles
    
    def setGoal(self, angles, goal):

        self.an = angles
        self.an1, self.an2, self.an3, self.an4, self.an5, self.an6, self.an7 = angles
        self.goal = goal
        self.tmpGoal = goal

    def update(self, speeds):

        self.v = speeds
        self.v1, self.v2, self.v3, self.v4, self.v5, self.v6 = speeds
        self.an = [angle + velocity * self.dt for angle, velocity in zip(self.an, speeds)]
        self.an1, self.an2, self.an3, self.an4, self.an5, self.an6, self.an7 = self.an + [0]
    
    def getManipulability(self):

        manipulability = np.sqrt(np.linalg.det(self.J@self.J.T))
        if manipulability < self.manipulability:
            self.manipulability=manipulability

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
            self.toolpose = self.getPose(FKMatrixList[6])

            H = FKMatrixList[6]

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

        self.getManipulability()

    def calculateJointSpeed(self, desired_end_speed):
        if self.J_inv is not None:
            desired_end_speed = np.array(desired_end_speed).reshape((-1,1))
            Jinv = self.J_inv
            joint_speed = (Jinv @ desired_end_speed).flatten()
            self.jointSpeed = joint_speed
            return joint_speed

        else:
            return None
    
    def getJointSpeed(self, jointspeed):
        
        #Use a controller to reach desired speed in a certain time i.e. 100 iterations
        #Achieved by a proportional controller - just P
        speed = [self.v[i] + ((jointspeed[i] - self.v[i])/self.K) for i in range(0,6)]

        #speed = [0,-0.1,0,0,0,0]
        self.jointSpeed = speed
        self.clampSpeed()   
        data = self.jointSpeed
        return data
    
    def getError(self,goal,scalar=False):

        goalMat = build_se3_transform(goal)
        goalMat = goalMat @ self.KinChain.invtool

        goal = self.getPose(goalMat)

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
    
    def reachPose(self, success_thresh = 0.05):

        success = False
        goal = self.tmpGoal

        #Use a similar controller to approach the desired coordinates for the end effector

        #Now we have to estimate the error on the EE, not on the tool

        if self.getError(goal, scalar=True) < success_thresh:

            data = [0,0,0,0,0,0]
            success = True
            #print("Reached goal with precision > 0.01")

        else:

            #TODO: Be able to calculate dt after each and every iteration / done?
            #PID Control
            #Normalize the speed vector
            #This is possible since the Jacobian matrix is a linear approximation of end-effector kinematics for pose.

            #error = np.array([(goal[i] - self.pose6[i]) for i in range(6)])
            #self.error = error

            #This is not as accurate as I thought, so let's remake it

            error = self.getError(goal,scalar=False)
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
            data = self.publishJointSpeed(jointspeed)
            success = False
            

        return data, success

    def publishJointSpeed(self, jointspeed):
        
        msg = None
        #Use a controller to reach desired speed in a certain time i.e. 100 iterations
        #Achieved by a proportional controller - just P
        speed = [self.v[i] + ((jointspeed[i] - self.v[i])/self.K) for i in range(0,6)]

        #speed = [0,-0.1,0,0,0,0]
        self.jointSpeed = speed
        self.clampSpeed()   
        msg = self.jointSpeed
        return msg
    
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
    
    def collisionCheckRequest(self):

        msg = PoseArray()
        msg.element_poses = [list2Pose(self.pose1), list2Pose(self.pose2), list2Pose(self.pose3), list2Pose(self.pose4), list2Pose(self.pose5), list2Pose(self.pose6), list2Pose(self.toolpose)]
        msg.id = self.request_id

        self.posePublisher.publish(msg)

    def checkerCallback(self, msg):

        if msg.success:

            self.collided = True
    
    def loop(self, success_thresh=0.05):

        success = False
        iterations = 1

        while not success:

            if (self.manipulability < self.manipulability_thresh) or self.collided:

                self.manipulability = 2
                self.collided = False
                return False, None

            self.calcEveryH()
            self.FK()
            if iterations % 20 == 0:
                self.collisionCheckRequest()
            self.calculateJ()
            data, success = self.reachPose(success_thresh)
            self.update(data)

            iterations += 1
            if iterations % 100 == 0:

                print(bcolors.BOLD + str(iterations) + bcolors.ENDC + " iterations went by")
                print(self.getError(self.tmpGoal, scalar=False))
            
        if len(self.an) == 6:
            self.an.append(0)
        
        return True, self.an

    def timedLoop(self, success_thresh=0.001, maxtime=999):

        success = False
        iterations = 1
        time_1 = time.time()
        time_2 = time.time()

        while not success:

            time_2 = time.time()
            if (time_2 - time_1) > maxtime:
                
                print(bcolors.FAIL + bcolors.BOLD + "IK Failed: total time consumed." + bcolors.ENDC)
                return False, None

            if self.collided:

                self.manipulability = 2
                self.collided = False
                print(bcolors.FAIL + bcolors.BOLD + "IK Failed: System collided." + bcolors.ENDC)
                return False, None

            self.calcEveryH()
            self.FK()
            if iterations % 20 == 0:
                self.collisionCheckRequest()
            self.calculateJ()
            data, success = self.reachPose(success_thresh)
            self.update(data)

            iterations += 1
            if iterations % 10 == 0:

                print(bcolors.BOLD + str(iterations) + bcolors.ENDC + " iterations went by")
                print(self.getError(self.tmpGoal, scalar=False))
            
            
            
        if len(self.an) == 6:
            self.an.append(0)
        
        return True, self.an

class IKSolver(ClosedLoopUR10e):

    __slots__ = {'requestSubscriber', 'anglePublisher', 'maxtime', 'success'}

    def __init__(self, request_sub_topic, response_pub_topic, sub_topic="/collision_requests", pub_topic="/collision_replies", sim=True, tool='none', manipulability_thresh = 0.01, maxtime = 10):

        super().__init__(sub_topic, pub_topic, sim, tool, manipulability_thresh)

        self.requestSubscriber = rospy.Subscriber(request_sub_topic, IKRequest, self.requestCallback, queue_size=1)
        self.anglePublisher = rospy.Publisher(response_pub_topic, IKReply, queue_size=1, latch=True)

        self.success = False
        self.maxtime = maxtime

    
    def requestCallback(self, msg):

        print(bcolors.OKGREEN + bcolors.BOLD + "Request received" + bcolors.ENDC)
        reply = IKReply()
        self.reset()
        self.setGoal(msg.start_angles, pose2Array(msg.end))
        self.maxtime = msg.maxtime
        self.success, angles = self.timedLoop(maxtime=self.maxtime)

        if self.success:
            reply.success = self.success
            reply.angles = angles
            self.anglePublisher.publish(reply)
            print(bcolors.OKGREEN + bcolors.BOLD + "Found reachable solution for the desired pose!" + bcolors.ENDC)
        
        else:

            print(bcolors.WARNING + bcolors.BOLD + "Unsuccessful in solving IK." + bcolors.ENDC)

    