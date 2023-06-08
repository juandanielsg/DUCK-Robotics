#!/usr/bin/python3

import numpy as np
import numpy.matlib as matlib
from math import sin, cos, atan2, sqrt, asin
from dataclasses import dataclass, replace, field, fields
from typing import TypedDict
from immortals_messages.msg import Pose, PoseArray, Reply
from std_msgs.msg import Bool
import rospy

nonzero = 1e-09

#Just for fancy terminal work
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


@dataclass
class KinematicChain():

    base: np.array = np.array([])
    homogeneousMatrixList: list = field(default_factory=list)
    tool: np.array = np.eye(4)
    invtool: np.array = np.eye(4)
    size: int = 0

    def setBase(self, Matrix):

        setattr(self, "base", Matrix)
    
    def setTool(self, Matrix):

        setattr(self, "tool", Matrix)
        setattr(self, "invtool", TransInv(Matrix))

    def append(self, Matrix) -> None:

        self.homogeneousMatrixList.append(Matrix)
        self.size += 1

    def remove(self, index) -> None:
        
        self.homogeneousMatrixList = self.homogeneousMatrixList[:index]
        self.size = index
    
    def getSize(self) -> int:

        return self.size
    
    def update(self, MatrixList) -> None:
        
        setattr(self, "homogeneousMatrixList", MatrixList)
        setattr(self, "size", len(MatrixList))
        
    def ForwardKinematics(self) -> list:
        
        currentHomogeneousTransformMatrix = self.base
        forwardKinematicMatrixList = []
        
        for element in self.homogeneousMatrixList:

            currentHomogeneousTransformMatrix = currentHomogeneousTransformMatrix @ element
            forwardKinematicMatrixList.append(currentHomogeneousTransformMatrix)
        
        
        currentHomogeneousTransformMatrix = currentHomogeneousTransformMatrix @ self.tool
        forwardKinematicMatrixList.append(currentHomogeneousTransformMatrix)
        
        return forwardKinematicMatrixList

    def ForwardKinematicsJointPose(self,index) -> np.array:

        currentHomogeneousTransformMatrix = self.base
        
        for i in range(index+1):
            currentHomogeneousTransformMatrix = currentHomogeneousTransformMatrix @ self.homogeneousMatrixList[i]

        return currentHomogeneousTransformMatrix


def list2Pose(poselist: list):

    pose = Pose()
    pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw = poselist
    return pose

def position2Pose(position: np.array):

    return np.append(position, np.zeros(3))



def TransToRp(T):
    """Converts a homogeneous transformation matrix into a rotation matrix
    and position vector

    :param T: A homogeneous transformation matrix
    :return R: The corresponding rotation matrix,
    :return p: The corresponding position vector.

    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        (np.array([[1, 0,  0],
                   [0, 0, -1],
                   [0, 1,  0]]),
         np.array([0, 0, 3]))
    """
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]

def TransInv(T):
    """Inverts a homogeneous transformation matrix

    :param T: A homogeneous transformation matrix
    :return: The inverse of T
    Uses the structure of transformation matrices to avoid taking a matrix
    inverse, for efficiency.

    Example input:
        T = np.array([[1, 0,  0, 0],
                        [0, 0, -1, 0],
                        [0, 1,  0, 3],
                        [0, 0,  0, 1]])
    Output:
        np.array([[1,  0, 0,  0],
                    [0,  0, 1, -3],
                    [0, -1, 0,  0],
                    [0,  0, 0,  1]])
    """
    R, p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]


def build_se3_transform(xyzrpy):
    """Creates an SE3 transform from translation and Euler angles.

    Args:
        xyzrpy (list[float]): translation and Euler angles for transform. Must have six components.

    Returns:
        numpy.matrixlib.defmatrix.matrix: SE3 homogeneous transformation matrix

    Raises:
        ValueError: if `len(xyzrpy) != 6`

    """
    if len(xyzrpy) != 6:
        raise ValueError("Must supply 6 values to build transform")

    se3 = matlib.identity(4)
    se3[0:3, 0:3] = euler_to_so3(xyzrpy[3:6])
    se3[0:3, 3] = np.matrix(xyzrpy[0:3]).transpose()
    return se3


def euler_to_so3(rpy):
    """Converts Euler angles to an SO3 rotation matrix.

    Args:
        rpy (list[float]): Euler angles (in radians). Must have three components.

    Returns:
        numpy.matrixlib.defmatrix.matrix: 3x3 SO3 rotation matrix

    Raises:
        ValueError: if `len(rpy) != 3`.

    """
    if len(rpy) != 3:
        raise ValueError("Euler angles must have three components")

    R_x = np.matrix([[1, 0, 0],
                     [0, cos(rpy[0]), -sin(rpy[0])],
                     [0, sin(rpy[0]), cos(rpy[0])]])
    R_y = np.matrix([[cos(rpy[1]), 0, sin(rpy[1])],
                     [0, 1, 0],
                     [-sin(rpy[1]), 0, cos(rpy[1])]])
    R_z = np.matrix([[cos(rpy[2]), -sin(rpy[2]), 0],
                     [sin(rpy[2]), cos(rpy[2]), 0],
                     [0, 0, 1]])
    R_zyx = R_z * R_y * R_x
    return R_zyx


def so3_to_euler(so3):


    MATRIX_MATCH_TOLERANCE = 1e-4
    """Converts an SO3 rotation matrix to Euler angles

    Args:
        so3: 3x3 rotation matrix

    Returns:
        numpy.matrixlib.defmatrix.matrix: list of Euler angles (size 3)

    Raises:
        ValueError: if so3 is not 3x3
        ValueError: if a valid Euler parametrisation cannot be found

    """
    if so3.shape != (3, 3):
        raise ValueError("SO3 matrix must be 3x3")
    roll = atan2(so3[2, 1], so3[2, 2])
    yaw = atan2(so3[1, 0], so3[0, 0])
    denom = sqrt(so3[0, 0] ** 2 + so3[1, 0] ** 2)
    pitch_poss = [atan2(-so3[2, 0], denom), atan2(-so3[2, 0], -denom)]

    R = euler_to_so3((roll, pitch_poss[0], yaw))

    if (so3 - R).sum() < MATRIX_MATCH_TOLERANCE:
        return np.matrix([roll, pitch_poss[0], yaw])
    else:
        R = euler_to_so3((roll, pitch_poss[1], yaw))
        if (so3 - R).sum() > MATRIX_MATCH_TOLERANCE:
            raise ValueError("Could not find valid pitch angle")
        return np.matrix([roll, pitch_poss[1], yaw])
    

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
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)

    return [roll_x, pitch_y, yaw_z] # in radians



#TODO: Light Hitbox class and derivates

class LightHitbox():

    __slots__ = {'diameter', 'radius', 'position'}

    def __init__(self, pose, diameter):

        self.diameter = diameter
        self.radius = diameter/2
        self.position = np.array(pose[:3])

    def distance(self, Hitbox2):

        return np.linalg.norm(self.position - Hitbox2.position)

    def collides(self, Hitbox2):

        return True if self.distance(Hitbox2) < (self.radius + Hitbox2.radius) else False
    
    def updatePosition(self, position):

        self.position = np.array(position)
    
    def __str__(self):

        return "Light hitbox. Origin: " + str(self.position) + " / Diameter: " + str(self.diameter)



class LightHitboxBodyLink():

    """
    This class allows the user to define simple body links, and has some handy methods for checking collisions
    """

    __slots__ = {'endPose', 'endPosition', 'startPose', 'startPosition', 'length', 'vector', 'width', 'label', 'ratio', 'hitboxes'}

    def __init__(self, startPose, endPose, width, label=0):

        self.endPose = endPose
        self.endPosition = endPose[:3]
        self.startPose = startPose
        self.startPosition = startPose[:3]
        self.length = np.linalg.norm(self.endPosition - self.startPosition)
        self.vector = (self.endPosition - self.startPosition)/self.length
        self.width = width
        self.label = label

        self.ratio = int(self.length * 4 / self.width)
        
        self.hitboxes = self.populateLink()
    
    def populateLink(self):

        point_poses = np.linspace(self.startPosition, self.endPosition, self.ratio)
        hitboxes = [LightHitbox(pose=position2Pose(point), diameter=self.width) for point in point_poses]

        return hitboxes
    
    def updateLink(self):

        point_poses = np.linspace(self.startPosition, self.endPosition, self.ratio)

        for i in range(len(point_poses)):

            self.hitboxes[i].updatePosition(point_poses[i])

    def updatePosition(self, start, end):

        self.startPosition = start[:3]
        self.endPosition = end[:3]
        self.updateLink()

    def collide(self, link2):

        for box in self.hitboxes:
            for box_ in link2.hitboxes:
                if box.collides(box_):
                    print(box)
                    print(box_)
                    return True

        return False
    
    def collideAdjacent(self, link2):
        """Only use this method for adjacent links"""

        box = self.hitboxes[-1]
        for box_ in link2.hitboxes:
            if box.collides(box_):
                print(box)
                print(box_)
                return True
            
        return False

    def __str__(self) -> str:
        
        return "Chain. Label: " + str(self.label) + " / Start: " + str(self.startPosition) + " / End: " + str(self.endPosition)

class LightHitboxGroup():

    """
    Call me Fernando Alonso the way I'm doing Magic here. (Lightweight, duh)
    """

    __slots__ = {'bodyChain', 'externalObjects', 'robotSubscriber', 'collisionPublisher'}

    def __init__(self,sub_topic, pub_topic):

        self.bodyChain = []
        self.externalObjects = [] #This is yet to be implemented
        self.init_()
        self.robotSubscriber = rospy.Subscriber(sub_topic, PoseArray, self.robotCallback, queue_size=1000)
        self.collisionPublisher = rospy.Publisher(pub_topic, Reply, queue_size=1000, latch=True)
        
        #TODO: Grab some values to initialize this chain. This should be easy. - DONE
        #TODO: Add a function to get MarkerArray from all - DONE
        #TODO: Add a function to initialize bodychains - DONE

    def init_(self):

        #Poses for starters
        poses = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], #Base
        [0.0, 0.0, 0.1807, 0, 0, 0], #joint1
        [0.004611027447044222, 3.9578729083808876e-05, 0.7933826477544528, 0, 0, 0], #joint 2
        [0.5732271504231331, 0.004920292136765654, 0.7357520497333189, 0, 0, 0], #joint 3
        [0.5717323899058908, 0.1790638771205258, 0.7357520497333189, 0, 0, 0], #wrist1
        [0.6915777911753764, 0.1800925694723422, 0.7355420816374054, 0, 0, 0], #wrist2
        [0.6913736393188675, 0.18008762839210732, 0.6189922605409953, 0, 0, 0], #wrist3
        [0.690497826978632, 0.18006643113670237, 0.1189930280381633, 0, 0, 0]] #tool

        poses = [np.array(pose) for pose in poses]
        widths = [0.56, 0.56, 0.25, 0.135, 0.135, 0.135, 0.05]
        labels = [2, 3, 1, 1, 1, 1, 5]
        posepairs = [(poses[i], poses[i+1], widths[i], labels[i]) for i in range(len(poses)-1)]
        self.bodyChain = [LightHitboxBodyLink(posepair[0], posepair[1], posepair[2], label=posepair[3]) for posepair in posepairs]

    def robotCallback(self,msg):
        poses = msg.element_poses
        poses = [np.array([pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw]) for pose in poses]
        posepairs = [(poses[i], poses[i+1]) for i in range(len(poses)-1)]
        [self.bodyChain[i+1].updatePosition(posepairs[i][0], posepairs[i][1]) for i in range(len(posepairs))] #This is a flex tbh
        self.solve(msg.id)
    
    def collisionCheck(self):
        
        chain = self.bodyChain
        index = len(chain)-1
        currentLink = chain[index]

        while index > 0:
            
            first = True

            for i in range(index-1, -1, -1):

                if currentLink.label != chain[i].label:

                    if first:

                        first = False
                        
                    else:

                        if currentLink.collide(chain[i]):
                            return True
                        
                else:

                    first = False
            
            
            index -= 1
            #print(index)
            currentLink = chain[index]
                
        return False
    
    def solve(self, id):
        """Solves self-collision in a fast yet precise way"""

        msg = Reply()
        msg.success = self.collisionCheck()
        msg.id = id
        self.collisionPublisher.publish(msg)


def getErrorVectors(pose,goal):

    linearerror = goal[:3] - pose[:3]

    w, x, y, z = get_quaternion_from_euler(goal[3],goal[4],goal[5])
    finalQt = np.quaternion(w, x, y, z)
    w, x, y, z = get_quaternion_from_euler(pose[3],pose[4],pose[5])
    Qt = np.quaternion(w,x,y,z)

    errorqt = finalQt * Qt.conjugate()

    trueEulError = np.array(euler_from_quaternion(errorqt.w, errorqt.x, errorqt.y, errorqt.z))
    #print(trueEulError)
    linearNorm = np.linalg.norm(linearerror)
    angularNorm = np.linalg.norm(trueEulError)

    error = linearNorm + angularNorm

    positionRatio = linearNorm/error
    eulerRatio = 1 - positionRatio

    linearerror /= (linearNorm + 1e-09)
    trueEulError /= (angularNorm + 1e-09)

    return [error, linearerror, trueEulError, positionRatio, eulerRatio]

def getCloserPoint(pose, linearVector, angularVector, ratio1, ratio2, thresh=0.5):

    linearIncrement = linearVector * (ratio1 * thresh)
    angularIncrement = angularVector * (ratio2 * thresh)

    pose_ = pose + np.append(linearIncrement, angularIncrement)
    return pose_

def getError(pose,goal):

    error = np.array([(goal[i] - pose[i]) for i in range(3)])

    
    w, x, y, z = get_quaternion_from_euler(goal[3],goal[4],goal[5])
    finalQt = np.quaternion(w, x, y, z)
    w, x, y, z = get_quaternion_from_euler(pose[3],pose[4],pose[5])
    Qt = np.quaternion(w,x,y,z)

    errorqt = finalQt * Qt.conjugate()
    trueEulError = np.array(euler_from_quaternion(errorqt.w, errorqt.x, errorqt.y, errorqt.z))

    error = np.linalg.norm(error) + np.linalg.norm(trueEulError)
    
    return error


def pose2Array(pose):

    return np.array([pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw])