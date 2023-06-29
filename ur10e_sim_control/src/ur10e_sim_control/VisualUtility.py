#!/usr/bin/python3

import rospy
import numpy as np
import numpy.matlib as matlib
from math import sin, cos, atan2, sqrt, asin
from dataclasses import dataclass, replace, field, fields
from typing import TypedDict
from geometry_msgs.msg import Pose
from immortals_messages.msg import EulerPose, EulerPoseArray, Path
from ur10e_sim_control.Utility import get_quaternion_from_euler, euler_from_quaternion, position2Pose, euler_to_so3
from visualization_msgs.msg import Marker, MarkerArray
import quaternion
from std_msgs.msg import Bool

class MarkerData():
    """Builder class for Marker messages, so it is easier to configure them.
    """

    def __init__(self, pose, shape=Marker.SPHERE, color = (1,1,0,0), size = (0.01, 0.01, 0.01)):

        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.id = 0
        self.marker.type = shape
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = pose[0]
        self.marker.pose.position.y = pose[1]
        self.marker.pose.position.z = pose[2]
        self.quaternion = get_quaternion_from_euler(pose[3],pose[4],pose[5])
        self.marker.pose.orientation.x = self.quaternion[1]
        self.marker.pose.orientation.y = self.quaternion[2]
        self.marker.pose.orientation.z = self.quaternion[3]
        self.marker.pose.orientation.w = self.quaternion[0]
        self.marker.scale.x = size[0]
        self.marker.scale.y = size[1]
        self.marker.scale.z = size[2]
        self.marker.color.a = color[0]
        self.marker.color.r = color[1]
        self.marker.color.g = color[2]
        self.marker.color.b = color[3]

        self.pose = pose
        self.reached = False

    def setPose(self, pose):

        self.marker.pose.position.x = pose[0]
        self.marker.pose.position.y = pose[1]
        self.marker.pose.position.z = pose[2]
        self.quaternion = get_quaternion_from_euler(pose[3],pose[4],pose[5])
        self.marker.pose.orientation.x = self.quaternion[1]
        self.marker.pose.orientation.y = self.quaternion[2]
        self.marker.pose.orientation.z = self.quaternion[3]
        self.marker.pose.orientation.w = self.quaternion[0]
    
    def setColor(self, color):

        self.marker.color.a = color[0]
        self.marker.color.r = color[1]
        self.marker.color.g = color[2]
        self.marker.color.b = color[3]
    
    def setSize(self, size):

        self.marker.scale.x = size[0]
        self.marker.scale.y = size[1]
        self.marker.scale.z = size[2]
    
    def setShape(self, shapetype):

        #WIP

        self.marker.type = shapetype
    
    def setID(self, integer):

        self.marker.id = integer
    
    def get(self):
        return self.marker

    def getError(self,goal):

        error = np.array([(goal[i] - self.pose[i]) for i in range(3)])

        w, x, y, z = get_quaternion_from_euler(goal[3],goal[4],goal[5])
        finalQt = np.quaternion(w, x, y, z)
        currentQt = np.quaternion(self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3])

        errorqt = finalQt * currentQt.conjugate()
        trueEulError = np.array(euler_from_quaternion(errorqt.w, errorqt.x, errorqt.y, errorqt.z))

        error = np.append(error, trueEulError)
        error = np.linalg.norm(error)
        
        return error

    def reach(self):
        self.reached = True

    def setPosition(self, position):

        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]       

class MarkerList():

    def __init__(self):

        self.markerList = []
        
    def getMessage(self):

        message = MarkerArray()
        id = 0
        for marker in self.markerList:

            marker.setID(id)
            id += 1
            message.markers.append(marker.get())
        
        return message

    def addMarker(self, marker: MarkerData):

        self.markerList.append(marker)
    
    def setMarkerList(self, markers):
        """Brute force method to easily connect MarkerList and Hitbox classes. Do not call otherwise.
        """

        self.markerList = markers

    def removeMarker(self, index):

        #WIP
        self.markerList.pop(index)

    def updateMarkers(self, currentPose):

        for marker in self.markerList:

            if not(marker.reached) and marker.getError(currentPose) < 0.05:

                marker.setColor([1,0,1,0])
                marker.reach()
    
    def set(self, newPoseList):

        self.clear()
        
        for i in range(len(newPoseList)):

            marker = MarkerData(pose=newPoseList[i])
            if i == len(newPoseList)-1:
                marker.setSize([0.04,0.04,0.04])
            self.addMarker(marker)
    
    def clear(self):

        self.markerList = []
    
    def concatenate(self, markerlist: MarkerArray):
        """Adds two markerlists together"""
        self.markerList += markerlist.markerList

class visualizationController():

    def __init__(self, subscriber_topic=None, visual_subscriber_topic=None, publisher_topic=None):

        if visual_subscriber_topic:
            self.visualizeSubscriber = rospy.Subscriber(visual_subscriber_topic, Pose, self.robotCallback)
        if subscriber_topic:
            self.pathSubscriber = rospy.Subscriber(subscriber_topic, Path, self.pathCallback)
        if publisher_topic:
            self.visualizePublisher = rospy.Publisher(publisher_topic, MarkerArray, latch=True, queue_size=1)

        self.constraintPublisher = rospy.Publisher("/visual_constraint", Marker, queue_size=1, latch=True)
        self.constraintSubscriber = rospy.Subscriber("/constraint", Pose, queue_size=1, callback=self.constraintCallback)

        self.baseVector = np.array([0,0,1])
        self.planeVector = np.array([0,0,1])
        self.constraint = None
        self.markers = MarkerList()
        self.currentPose = None
        self.goalExists = False
        self.constraintExists = False
    
    def constraintCallback(self, msg):

        self.constraint = MarkerData(np.array([msg.position.x, msg.position.y, msg.position.z,0.0,0.0,0.0]), color=(1,1,0,0), size=(0.05,0.05,0.05))

        message = self.constraint.get()
        self.constraintExists = True

        mat = euler_to_so3(self.constraint.pose[3:])
        self.planeVector = mat @ self.baseVector

        self.constraintPublisher.publish(message)

    def robotCallback(self, msg):

        quat = euler_from_quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        self.currentPose = [msg.position.x, msg.position.y, msg.position.z] + quat

        self.markers.updateMarkers(self.currentPose)
        if self.goalExists:
            self.publishMarkers()

    def pathCallback(self, msg):

        data = [[elem.x, elem.y, elem.z, elem.roll, elem.pitch, elem.yaw] for elem in msg.trajarray]

        self.markers.set(data)
        self.goalExists = True
    
    def publishMarkers(self):

        msg = self.markers.getMessage()
        self.visualizePublisher.publish(msg)
    
    def printData(self):
        print("Current pose: ", self.currentPose)
        print("Current path: ", self.markers.markerList)

    def updateConstraint(self, pose):

        """Update when axis aligned"""

        mat = euler_to_so3(pose[3:])
        ee_vector = mat @ self.baseVector

        if np.linalg.norm(np.cross(ee_vector, self.planeVector)) <= 0.01:

            self.constraint.setColor((1,1,0,1))

        else:

            self.constraint.setColor((1,1,0,0))

        msg = self.constraint.get()
        self.constraintPublisher.publish(msg)            

class Hitbox(MarkerData):
    """Class for collision avoidance. Assumes all hitboxes will be spheres.
    """

    def __init__(self, pose, shape=Marker.SPHERE, color = (0.1,1,0,0), size = (0.01, 0.01, 0.01)):

        super().__init__(pose, shape, color, size)
        self.diameter = size[0]
        self.radius = self.diameter/2
        self.position = np.array(pose[:3])

    
    def distance(self, Hitbox2):

        return np.linalg.norm(self.position - Hitbox2.position)

    def collides(self, Hitbox2):

        return True if self.distance(Hitbox2) < (self.radius + Hitbox2.radius) else False
    
    def updatePosition(self, position):

        self.position = np.array(position)
        self.setPosition(self.position)
    
    def displayCollision(self):
        self.setColor([0.3,1,0,0])
    
    def __str__(self):
        return "Hitbox. Origin: " + str(self.position) + " / Radius: " + str(self.radius)
    
class HitboxBodyLink():

    """
    This class allows the user to define simple body links, and has some handy methods for checking collisions
    """

    def __init__(self, startPose, endPose, width, label):

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

        self.visualhitboxes = MarkerList()
        self.visualhitboxes.setMarkerList(self.hitboxes)
    
    def populateLink(self):

        point_poses = np.linspace(self.startPosition, self.endPosition, self.ratio)
        hitboxes = [Hitbox(pose=position2Pose(point), color=(0.3,0,0,1), size=(self.width, self.width, self.width)) for point in point_poses]

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
                    box.displayCollision()
                    box_.displayCollision()
                    return True

        return False

    def __str__(self) -> str:
        
        return "Chain. Label: " + str(self.label) + " / Start: " + str(self.startPosition) + " / End: " + str(self.endPosition)

class HitboxGroup():

    """
    Call me Fernando Alonso the way I'm doing Magic here.
    """

    __slots__ = {'bodyChain', 'externalObjects', 'visualPublisher', 'robotSubscriber', 'collisionPublisher', 'tool'}

    def __init__(self,sub_topic, visual_pub_topic, pub_topic, tool=None):

        self.bodyChain = []
        self.externalObjects = [] #This is yet to be implemented
        self.visualPublisher = rospy.Publisher(visual_pub_topic, MarkerArray, queue_size=1, latch=True)
        


        self.init_(tool)
        self.robotSubscriber = rospy.Subscriber(sub_topic, EulerPoseArray, self.robotCallback)
        self.collisionPublisher = rospy.Publisher(pub_topic, Bool, queue_size=1)
        
        self.tool = tool
        

        #TODO: Grab some values to initialize this chain. This should be easy.
        #TODO: Add a function to get MarkerArray from all - DONE
        #TODO: Add a function to initialize bodychains

    def init_(self, tool):

        #Poses for starters
        if tool == "cylinder1":
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
            labels = [2,3,1,1,1,1,1]
        
        else:
            poses = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], #Base
            [0.0, 0.0, 0.1807, 0, 0, 0], #joint1
            [0.004611027447044222, 3.9578729083808876e-05, 0.7933826477544528, 0, 0, 0], #joint 2
            [0.5732271504231331, 0.004920292136765654, 0.7357520497333189, 0, 0, 0], #joint 3
            [0.5717323899058908, 0.1790638771205258, 0.7357520497333189, 0, 0, 0], #wrist1
            [0.6915777911753764, 0.1800925694723422, 0.7355420816374054, 0, 0, 0], #wrist2
            [0.6913736393188675, 0.18008762839210732, 0.6189922605409953, 0, 0, 0]] #wrist3
            poses = [np.array(pose) for pose in poses]
            widths = [0.56, 0.56, 0.25, 0.135, 0.135, 0.135]
            labels = [2,3,1,1,1,1]

        posepairs = [(poses[i], poses[i+1], widths[i], labels[i]) for i in range(len(poses)-1)]
        self.bodyChain = [HitboxBodyLink(posepair[0], posepair[1], posepair[2], label=posepair[3]) for posepair in posepairs]
        self.display()

    def robotCallback(self,msg):
        poses = msg.element_poses
        poses = [np.array([pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw]) for pose in poses]
        posepairs = [(poses[i], poses[i+1]) for i in range(len(poses)-1)]
        #print(len(posepairs))
        if self.tool:
            [self.bodyChain[i+1].updatePosition(posepairs[i][0], posepairs[i][1]) for i in range(len(posepairs))] #This is a flex tbh
        
        else:
            [self.bodyChain[i+1].updatePosition(posepairs[i][0], posepairs[i][1]) for i in range(len(posepairs)-1)]
        #self.display()
    
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

    def display(self):

        hitboxes = MarkerList()

        for i in range(len(self.bodyChain)):

            hitboxes.concatenate(self.bodyChain[i].visualhitboxes)
        
        message = hitboxes.getMessage()
        self.visualPublisher.publish(message)

    def solve(self):
        """Solves self-collision in a fast yet precise way"""

        msg = Bool()
        msg.data = self.collisionCheck()
        self.collisionPublisher.publish(msg)