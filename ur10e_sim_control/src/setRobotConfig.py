#!/usr/bin/python3

import rospy
import os
from gazebo_msgs.srv import SetModelConfiguration
from ur10e_sim_control.Utility import bcolors

def setconfig_client(message):
    rospy.wait_for_service('/gazebo/set_model_configuration')
    try:
        setconfig = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        name, _, jointnames, jointangles = message
        resp1 = setconfig(name, _, jointnames, jointangles)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():

    rospy.init_node("setConfigurationNode")
    message = [None, None, None, None]
    
    message[0] = 'robot'
    message[2] = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

    keywords = {'Home': [0,-1.5707,1.5707,-1.5707,-1.5707,0]}


    while not rospy.is_shutdown():
        os.system("clear")
        print(bcolors.BOLD + bcolors.OKGREEN + "Set the end effector angles manually here!" + bcolors.ENDC)
        print("Set a list of angles for the robot configuration or use a keyword: ")
        print("Saved keywords: ", keywords.keys())
        user_input = input("Write here (separate joint radian angles by commas): ")

        if user_input in keywords.keys():

            message[3] = keywords[user_input]
            setconfig_client(message)
        
        else:

            try:

                user_input = [float(datum) for datum in user_input.split(",")]
                if len(user_input) == 6:

                    message[3] = user_input
                    setconfig_client(message)
                
                else:

                    print("ERROR: Insufficient angles defined!")
            
            except:

                print("ERROR: Wrong format!")
        
if __name__ == "__main__":

    main()