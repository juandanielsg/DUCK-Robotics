import numpy as np
import numpy.matlib as matlib
from math import sin, cos, atan2, sqrt, asin
from dataclasses import dataclass, replace, field, fields
from typing import TypedDict
import rospy
from immortals_messages.msg import PoseArray, Reply, Path, Pose, PathRequest
from ur10e_sim_control.Utility import getErrorVectors, getCloserPoint, bcolors, pose2Array, getError, list2Pose
from ur10e_sim_control.PlanningHelper import ClosedLoopUR10e
import time
from ur10e_sim_control.Utility import bcolors

class Planner():

    """
    Base class for path planning. Uses RRT*.
    """

    __slots__ = {'requestSubscriber', 'pathPublisher', 'low_limits', 'top_limits', 'nodes', 'threshold', 'radius_threshold', 'ispositional', 
                 'start_node', 'end_node', 'max_time', 'max_nodes_expanded', 'manipulability_threshold', 'cost_threshold', 'planningHelper'}

    def __init__(self, sub_topic="/path_requests", pub_topic="/plan_responses", max_time = 9999, max_nodes_expanded = 100000, manipulability_threshold = 0.1, tool = "none"):


        #ROS Topics - Layer 1
        self.requestSubscriber = rospy.Subscriber(sub_topic, PathRequest, self.trajectoryCallback, queue_size=1)
        self.pathPublisher = rospy.Publisher(pub_topic, Path, latch=True, queue_size=1)

        #The idea here is getting the optimal path with some constraints
        self.low_limits = np.array([-1,-1,0,-3.14,-3.14,-3.14])
        self.top_limits = np.array([1,1,1,3.14,3.14,3.14])

        self.nodes = []

        self.threshold = 0.5
        self.radius_threshold = 0.5
        self.ispositional = False

        self.start_node = State()
        self.end_node = State()
        self.max_time = max_time
        self.max_nodes_expanded = max_nodes_expanded
        self.manipulability_threshold = manipulability_threshold

        self.cost_threshold = None
        self.planningHelper = ClosedLoopUR10e(sub_topic="/collision_requests", pub_topic="/collision_replies", tool=tool,manipulability_thresh=manipulability_threshold)
    
    def trajectoryCallback(self, msg):

        print(bcolors.BOLD + "Plan request received." + bcolors.ENDC)

        self.nodes = []
        self.ispositional = False
        self.start_node.setPose(pose2Array(msg.start))
        self.start_node.setAngles(msg.start_angles)
        self.end_node.setPose(pose2Array(msg.end))

        if self.getErrorVectors(self.start_node, self.end_node)[3] > 0.99:

            print(print(bcolors.BOLD + "Detected as position planning" + bcolors.ENDC))
            self.ispositional = True

        self.cost_threshold = self.getDistance(self.start_node, self.end_node) * 3

        self.max_time = msg.maxtime
        self.nodes.append(self.start_node)
        msg = self.search()

        if msg.success:

            print(bcolors.BOLD + bcolors.OKGREEN + "Planning succeeded!" + bcolors.ENDC)
    
        else:

            print(bcolors.BOLD + bcolors.WARNING + "Planning failed!" + bcolors.ENDC)

        self.pathPublisher.publish(msg)

    def getRandomNode(self):

        randomNode = State()
        if self.ispositional:
            randomNode.setPose(np.append(np.round(np.random.uniform(low=self.low_limits[:3], high=self.top_limits[:3], size=3,), 3), self.start_node.pose[3:]))
        else:
            randomNode.setPose(np.round(np.random.uniform(low=self.low_limits, high=self.top_limits, size=6,), 3))
        return randomNode
    
    @staticmethod
    def getDistance(node, node_2):

        return getError(node.pose, node_2.pose)

    @staticmethod
    def getErrorVectors(node, node_2):

        return getErrorVectors(node.pose, node_2.pose)
    
    def trajectoryCheck(self, node, node_2):

        self.planningHelper.reset()
        self.planningHelper.setGoal(node.angles, node_2.pose)
        success, angles = self.planningHelper.loop()
        
        
        if success:

            node_2.angles = angles
            return True
        
        else:

            return False
    
    def getNearest(self, newnode):

        minimal = 99999
        nearest = None

        for node in self.nodes:

            dst = self.getDistance(node, newnode)
            if  dst < minimal:

                minimal = dst
                nearest = node
        
        return nearest
    
    def getCloserNode(self, node1, node2):

        error = self.getErrorVectors(node1, node2)

        if error[0] > self.threshold:

            node2.pose = getCloserPoint(node1.pose, error[1], error[2], error[3], error[4], thresh=self.threshold)
        
        return node2

    def checkRadius(self, newnode):

        bestnode = newnode.parent

        for node in self.nodes:

            if self.getDistance(node, newnode) < self.radius_threshold and node.total_cost + self.getDistance(node,newnode) < bestnode.total_cost + self.getDistance(bestnode,newnode) and self.trajectoryCheck(node, newnode):

                newnode.rewire(node)
        
    def rewire(self, newnode):

        for node in self.nodes:

            if node is not newnode.parent and self.getDistance(node, newnode) < self.threshold and newnode.total_cost + self.getDistance(node, newnode) < node.total_cost and self.trajectoryCheck(node, newnode):

                node.rewire(newnode)

    def constructPath(self):

        
        current_node = self.end_node
        pathlist = [current_node.pose]

        while current_node is not self.start_node:

            current_node = current_node.parent
            pathlist.append(current_node.pose)
        
        return pathlist

    def search(self):

        msg = Path()
        pathlist = [None]
        success = False
        iterations = 0
        time_1 = time.time()
        time_2 = time.time()

        while (time_2 - time_1) < self.max_time and iterations < self.max_nodes_expanded:
            
            newnode = self.getRandomNode()

            closest_node = self.getNearest(newnode)

            newnode = self.getCloserNode(closest_node, newnode)

            

            if (self.getDistance(closest_node, newnode) + closest_node.total_cost + self.getDistance(newnode, self.end_node)) < self.cost_threshold: 
                
                if self.trajectoryCheck(closest_node, newnode):

                    newnode.rewire(closest_node)
                    #self.checkRadius(newnode)
                    #self.rewire(newnode)

                    self.nodes.append(newnode)

                    if self.getDistance(newnode, self.end_node) < self.threshold:

                        if self.trajectoryCheck(newnode, self.end_node):

                            self.end_node.rewire(newnode)
                            pathlist = self.constructPath()
                            success = True
                            break

            time_2 = time.time()
            iterations += 1

            if iterations % 10 == 0:

                print(bcolors.BOLD + str(iterations) + bcolors.ENDC + " nodes expanded in " + bcolors.BOLD + str(time_2-time_1) + bcolors.ENDC + " s." )
                print(bcolors.BOLD + str(len(self.nodes)) + bcolors.ENDC + " active nodes ")

        msg.success = success
        if success:
            msg.trajarray = [list2Pose(path) for path in pathlist[::-1]]
        return msg
    

class State():

    __slots__ = {'parent', 'pose', 'manipulability', 'cost', 'total_cost', 'angles'}

    def __init__(self, parent=None, pose=None, manipulability=None):

        self.parent = parent
        self.pose = pose
        self.manipulability = manipulability
        self.cost = 0
        self.total_cost = 0
        self.angles = None
    
    def setAngles(self, angles):

        self.angles = angles
    
    def setup(self, parent, pose):

        self.parent = parent
        self.pose = pose

    def setPose(self, pose):
        self.pose = np.asarray(pose)

    def setManipulability(self, manipulability):
        self.manipulability = manipulability
        
    def rewire(self, parent):

        self.parent = parent
        self.cost = getError(self.pose, self.parent.pose)
        self.total_cost = self.cost + self.parent.total_cost

    def setCost(self):

        self.cost = getError(self.pose, self.parent.pose)
        self.total_cost = self.cost + self.parent.total_cost