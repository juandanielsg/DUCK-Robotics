#!/usr/bin/python3

import rospy
from ur10e_sim_control.Utility import LightHitboxGroup

def main():

    rospy.init_node("CollisionNode")
    rate = rospy.Rate(10)
    #hitboxController = LightHitboxGroup(sub_topic="/current_poses", pub_topic="/nothing_2_see_here")

    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == "__main__":
    main()