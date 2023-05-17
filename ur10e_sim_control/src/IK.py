#!/usr/bin/env python3

import rospy
import std_msgs, gazebo_msgs, trajectory_msgs, sensor_msgs
import numpy as np
import math

# Deprecated code

"""def fastFK(self):
    #Direct calculation of end effector position

    c1, c2, c3, c4, c5, c6 = [cos(x) for x in [self.an1, self.an2, self.an3, self.an4, self.an5, self.an6]]
    s1, s2, s3, s4, s5, s6 = [sin(x) for x in [self.an1, self.an2, self.an3, self.an4, self.an5, self.an6]]
    c23 = cos(self.an2+self.an3)
    s23 = sin(self.an2+self.an3)
    c234 = cos(self.an2+self.an3+self.an4)
    s234 = sin(self.an2+self.an3+self.an4)

    
    H00 = c234 * c1 * s5 - c5*s1
    H01 = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6
    H02 = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6
    H03 = self.d6*c234*c1*s5 - self.a3*c23*c1 - self.a2*c1*c2 - self.d6*c5*s1 - self.d5*s234*c1 - self.d4*s1
    H10 = c1*c5 + c234*s1*s5
    H11 = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6
    H12 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1
    H13 = self.d6*(c1*c5 + c234*s1*s5) + self.d4*c1 - self.a3*c23*s1 - self.a2*c2*s1 - self.d5*s234*s1
    H20 = -s234*s5
    H21 = -c234*s6 - s234*c5*c6
    H22 = s234*c5*s6 - c234*c6 
    H23 = self.d1 + self.a3*s23 + self.a2*s2 - self.d5*(c23*c4 - s23*s4) - self.d6*s5*(c23*s4 + s23*c4)


    H = np.array([[H00, H01, H02, H03],[H10, H11, H12, H13],[H20, H21, H22, H23],[0, 0, 0, 1]])
    #print(H)

    P = atan2(-H[2,0],sqrt(H[2,1]**2+H[2,2]**2))
    R = atan2(H[2,1]/cos(P), H[2,2]/cos(P))
    Y = atan2(H[1,0]/cos(P), H[0,0]/cos(P))

    #print([H[0,3], H[1,3], H[2,3], R, P, Y])
    pass"""

"""def calcEveryH(self):
    if self.an1 != None:
        self.H1 = self.calcH(1)
        self.H2 = self.calcH(2)
        self.H3 = self.calcH(3)
        self.H4 = self.calcH(4)
        self.H5 = self.calcH(5)
        self.H6 = self.calcH(6)

        self.H_list = [self.messySolution(), self.H1, self.H2, self.H3, self.H4, self.H5, self.H6]"""


"""    def calcH(self,n, debug = False):
        


        H = np.array([[cos(self.an[n-1]), -sin(self.an[n-1])*cos(self.alpha[n-1]), sin(self.an[n-1])*sin(self.alpha[n-1]), self.a[n-1]*cos(self.an[n-1])],[sin(self.an[n-1]), cos(self.an[n-1])*cos(self.alpha[n-1]), -cos(self.an[n-1])*sin(self.alpha[n-1]), self.a[n-1]*sin(self.an[n-1])],[0, sin(self.alpha[n-1]), cos(self.alpha[n-1]), self.d[n-1]],[0, 0, 0, 1]])


        if debug:

            print("Joint number: " + str(n))

            print("DH Params")
            print("a: " + str(self.a[n-1]))
            print("alpha: " + str(self.alpha[n-1]))
            print("d: " + str(self.d[n-1]))
            print("theta: " + str(self.an[n-1]))
            print()
            print(H)
        
        return H"""


"""def printDebug(self, flag = ""): #Deprecated
        if flag == "H":
            if len(self.H_list) != 0:
                print("~ FK ~")
                for element in self.H_list:
                    print('\n'.join(['\t'.join([str(round(cell,3)) for cell in row]) for row in element]))
                    print()
                print("~ FK 2 ~")
                for elem in self.H_list_2:
                    print('\n'.join(['\t'.join([str(round(cell,3)) for cell in row]) for row in elem]))
                    print()

        elif flag == "Pose":
            if self.F_H is not None:
                print("~ FK ~")
                print('\n'.join(['\t'.join([str(round(cell,3)) for cell in row]) for row in self.F_H]))
                print()
                print("~ FK 2 ~")
                FH2 = self.H_list_2[0] @ self.H_list_2[1] @ self.H_list_2[2] @ self.H_list_2[3] @ self.H_list_2[4] @ self.H_list_2[5] @ self.H_list_2[6]
                print('\n'.join(['\t'.join([str(round(cell,3)) for cell in row]) for row in FH2]))
                print()"""