import rospy
import os
from ur10e_sim_control.VisualUtility import visualizationController, HitboxGroup
from ur10e_sim_control.Utility import bcolors
from geometry_msgs.msg import Pose
import numpy

def definePoint(xyzrpy=None):

    point = Pose()
    point.position.x = xyzrpy[0]
    point.position.y = xyzrpy[1]
    point.position.z = xyzrpy[2]
    return point

def main(pub):

    rospy.init_node('constraint_setter', anonymous=True)

    while not rospy.is_shutdown():

        data = str(input("Set the desired constraint pose (XYZrpy), separated by commas: "))

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