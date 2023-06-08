import rospy
from ur10e_sim_control.PlanningHelper import IKSolver
from ur10e_sim_control.Utility import bcolors

def main():

    rospy.init_node("IK_Solver")
    rate = rospy.Rate(1)
    sub_topic = "/collision_requests"
    pub_topic = "/collision_replies"
    request_topic = "/IK_requests"
    reply_topic = "/IK_replies"
    tool = rospy.get_param('tool')
    solver = IKSolver(request_sub_topic=request_topic, response_pub_topic=reply_topic,sub_topic=sub_topic, pub_topic=pub_topic, tool = tool)
    print(bcolors.BOLD + bcolors.OKGREEN + "IK solver properly initialized!" + bcolors.ENDC)
    print(bcolors.OKGREEN + "Subscribed to topic " + bcolors.OKBLUE + bcolors.UNDERLINE + request_topic + bcolors.ENDC)
    print(bcolors.OKGREEN + "Publishing to topic " + bcolors.OKBLUE + bcolors.UNDERLINE + reply_topic + bcolors.ENDC)
    
    while not rospy.is_shutdown():

        rate.sleep()

if __name__ == "__main__":

    main()
