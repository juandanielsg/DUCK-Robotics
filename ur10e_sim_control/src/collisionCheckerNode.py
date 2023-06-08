#!/usr/bin/python3

import rospy
from ur10e_sim_control.Utility import LightHitboxGroup, bcolors

def main():

    rospy.init_node("CollisionNode")
    #rate = rospy.Rate(10)
    sub_topic = "/collision_requests"
    pub_topic = "/collision_replies"
    hitboxController = LightHitboxGroup(sub_topic=sub_topic, pub_topic=pub_topic)
    print(bcolors.BOLD + bcolors.OKGREEN + "Collision controller properly initialized!" + bcolors.ENDC)
    print(bcolors.OKGREEN + "Subscribed to topic " + bcolors.OKBLUE + bcolors.UNDERLINE + sub_topic + bcolors.ENDC)
    print(bcolors.OKGREEN + "Publishing to topic " + bcolors.OKBLUE + bcolors.UNDERLINE + pub_topic + bcolors.ENDC)
    

    while not rospy.is_shutdown():

        continue


if __name__ == "__main__":
    main()