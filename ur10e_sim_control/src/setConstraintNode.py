import rospy
import os
from ur10e_sim_control.VisualUtility import visualizationController, HitboxGroup
from ur10e_sim_control.Utility import bcolors
from geometry_msgs.msg import Pose
import numpy

def definePoint(xyzrpy):

    point = Pose()
    point.x = xyzrpy[0]
    point.y = xyzrpy[1]
    point.z = xyzrpy[2]
    point.roll = xyzrpy[3]
    point.pitch = xyzrpy[4]
    point.yaw = xyzrpy[5]

    return point

def main(pub):

    rospy.init_node('constraint_setter', anonymous=True)

    while not rospy.is_shutdown():

        data = str(input("Set the desired goal, separated by commas: "))

        data = [float(datum) for datum in data.split(",")]
        print(data)

        constraint = definePoint(data)

        pub.publish(constraint)




if __name__ == '__main__':

    try:

        pub = rospy.Publisher('/constraint', Pose, queue_size=1, latch=True)

        main(pub)

    except rospy.ROSInterruptException:
        pass
