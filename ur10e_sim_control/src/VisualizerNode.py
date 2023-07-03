#!/usr/bin/python3

import rospy
import os
from ur10e_sim_control.VisualUtility import visualizationController, HitboxGroup
from ur10e_sim_control.Utility import bcolors
import numpy as np



def main():

    rospy.init_node("VisualizerNode")

    tool = rospy.get_param('tool')
    controller = visualizationController(visual_subscriber_topic="/current_pose", subscriber_topic="/path_plan", publisher_topic="/visualization_marker_array")
    cntrl = rospy.get_param('controller_type')
    if cntrl != "jointP":
        hitboxController = HitboxGroup(sub_topic="/current_poses", pub_topic="/nothing_2_see_here", visual_pub_topic="/visualization_hitboxes", tool=tool)

    while not rospy.is_shutdown():
        
        os.system("clear")
        print(bcolors.OKGREEN + "Visuals rendered properly" + bcolors.ENDC)
        #if cntrl != "jointP":
            #hitboxController.collisionCheck()
            #hitboxController.display()
        

        if controller.constraintExists:
            #controller.updateConstraint(np.array(controller.currentPose))
            if controller.currentToolPos is not None and controller.currentEEPos is not None:

                print("Current RCM error (mm): " + str(controller.getRCMError()*1000))



        
        #rate.sleep()


if __name__ == "__main__":
    main()        