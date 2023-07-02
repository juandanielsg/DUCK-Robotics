#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
import PyKDL as kdl
from ur10e_sim_control.PlanningHelper import ClosedLoopUR10e
import ur10e_sim_control.urdf as urdf
import rospkg
import numpy as np
import os
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import pathlib

class ControllerGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        #Mundane variables
        self.var = 0.0
        self.rcm = tk.BooleanVar()
        self.message = Twist()
        self.rcm_message = Bool()
        self.pub = rospy.Publisher('/goal_speed', Twist, queue_size=10)
        self.rcmpub = rospy.Publisher('/toggle_rcm', Bool, queue_size=10)

        # configure the root window
        self.title('DUCK v0.7.2')
        self.geometry('720x480')


        
        #This does not work yet
        self.location = str(pathlib.Path(__file__).parent.resolve()) + "/gui/"
        self.icon = '@' + self.location + 'duck_logo.xbm'
        self.iconbitmap(self.icon)

        # label
        self.label = ttk.Label(self, text='Velocities')
        self.label.pack()

        # button
        self.button = ttk.Button(self, text='+Y')
        self.button['command'] = self.positive_y
        self.button.place(x=150,y=200)

        self.button2 = ttk.Button(self, text='-Y')
        self.button2['command'] = self.negative_y
        self.button2.place(x=150,y=300)

        self.button3 = ttk.Button(self, text='-X')
        self.button3['command'] = self.negative_x
        self.button3.place(x=50,y=250)

        self.button4 = ttk.Button(self, text='+X')
        self.button4['command'] = self.positive_x
        self.button4.place(x=250,y=250)

        self.button5 = ttk.Button(self, text='STOP')
        self.button5['command'] = self.STOP
        self.button5.place(x=150,y=250)

        self.button6 = ttk.Button(self, text='+Z')
        self.button6['command'] = self.positive_z
        self.button6.place(x=600,y=200)

        self.button7 = ttk.Button(self, text='-Z')
        self.button7['command'] = self.negative_z
        self.button7.place(x=600,y=300)

        self.scale1 = ttk.Scale(from_=0.001, to=0.05, command=self.scroller, length=200)
        self.scale1.place(x=500, y=100)

        self.value_label = ttk.Label(self, text="OFF")
        self.value_label.place(x=500, y=75)

        #RCM Toggle
        self.rcmbox = tk.Checkbutton(self, text='RCM', variable=self.rcm, onvalue=True, offvalue=False, command=self.toggleRCM)
        self.rcmbox.place(x=500, y=125)

        

    def positive_x(self):

        self.stopMessage()
        self.message.linear.x = self.var
        self.pub.publish(self.message)

    def negative_x(self):

        self.stopMessage()
        self.message.linear.x = -self.var
        self.pub.publish(self.message)

    def positive_y(self):
        
        self.stopMessage()
        self.message.linear.y = self.var
        self.pub.publish(self.message)

    def negative_y(self):
        
        self.stopMessage()
        self.message.linear.y = -self.var
        self.pub.publish(self.message)

    def positive_z(self):

        self.stopMessage()
        self.message.linear.z = self.var
        self.pub.publish(self.message)

    def negative_z(self):

        self.stopMessage()
        self.message.linear.z = -self.var
        self.pub.publish(self.message)

    def STOP(self):

        self.stopMessage()
        self.pub.publish(self.message)


    def stopMessage(self):

        self.message.linear.x = 0.0
        self.message.linear.y = 0.0
        self.message.linear.z = 0.0
        self.message.angular.x = 0.0
        self.message.angular.y = 0.0
        self.message.angular.z = 0.0
    

    def scroller(self, value):
        self.var = float(value)
        self.value_label["text"] = "Magnitude: " + "{:.5f}".format(self.var)

    def toggleRCM(self):
        self.rcm_message.data = self.rcm.get()
        self.rcmpub.publish(self.rcm_message)
        
        



        

if __name__ == "__main__":
    rospy.init_node("tkinter_gui")
    app = ControllerGUI()
    app.mainloop()