#!/usr/bin/python3

import rospy
import os
from ur10e_sim_control.VisualUtility import visualizationController, HitboxGroup
from ur10e_sim_control.Utility import bcolors



def main():

    rospy.init_node("VisualizerNode")
    rate = rospy.Rate(2)
    tool = rospy.get_param('tool')
    controller = visualizationController(visual_subscriber_topic="/current_pose", subscriber_topic="/path_plan", publisher_topic="/visualization_marker_array")
    hitboxController = HitboxGroup(sub_topic="/current_poses", pub_topic="/nothing_2_see_here", visual_pub_topic="/visualization_hitboxes", tool=tool)

    while not rospy.is_shutdown():
        
        os.system("clear")
        
        hitboxController.collisionCheck()
        hitboxController.display()
        print(bcolors.OKGREEN + "Visuals rendered properly" + bcolors.ENDC)
        rate.sleep()


if __name__ == "__main__":
    main()        