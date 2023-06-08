import rospy
from ur10e_sim_control.Planning import Planner
from ur10e_sim_control.Utility import bcolors

def main():

    rospy.init_node("PlanningNode")
    rate = rospy.Rate(1)
    sub_topic = "/path_requests"
    pub_topic = "/path_responses"
    planner = Planner(sub_topic=sub_topic, pub_topic=pub_topic, tool = 'none')
    print(bcolors.BOLD + bcolors.OKGREEN + "Planning controller properly initialized!" + bcolors.ENDC)
    print(bcolors.OKGREEN + "Subscribed to topic " + bcolors.OKBLUE + bcolors.UNDERLINE + sub_topic + bcolors.ENDC)
    print(bcolors.OKGREEN + "Publishing to topic " + bcolors.OKBLUE + bcolors.UNDERLINE + pub_topic + bcolors.ENDC)
    

    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == "__main__":
    main()