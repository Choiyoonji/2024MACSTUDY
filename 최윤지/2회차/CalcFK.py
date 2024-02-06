#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from math import *
import numpy as np
from geometry_msgs import Point

class CalcFK:
    def __init__(self):
        self.d0 = 0
        self.alpha0 = pi/2 
        self.a1 = 0
        self.a2 = 0
    
    def Calc(self, theta0, theta1, theta2):
        C0 = cos(theta0)
        S0 = sin(theta0)
        C1 = cos(theta1)
        S1 = sin(theta1)
        C2 = cos(theta2)
        S2 = sin(theta2)
        
        T01 = np.array([C0, -S0*C0, S0*sin(self.alpha0), 0],
                       [S0, C0*cos(self.alpha0), -C0*sin(self.alpha0), 0],
                       [0, sin(self.alpha0), cos(self.alpha0), self.d0],
                       [0, 0, 0, 1])
        
        T12 = np.array([C1, -S1*C1, S1*sin(0), self.a1*C1],
                       [S1, C1*cos(0), -C1*sin(0), self.a1*S1],
                       [0, sin(0), cos(0), 0],
                       [0, 0, 0, 1])
        
        T2H = np.array([C2, -S2*C2, S2*sin(0), self.a2*C2],
                       [S2, C2*cos(0), -C2*sin(0), self.a2*S2],
                       [0, sin(0), cos(0), 0],
                       [0, 0, 0, 1])
        
        T0H = T01 @ T12 @ T2H
        
        return T0H
    

class ThetaSub:
    def __init__(self):
        self.DH = CalcFK()
        pose = rospy.Subscriber('/pose', Point, self.callback_rot, queue_size=1)
        
    def callback_pose(self, msg):
        self.DH.Calc(msg.x, msg.y, msg.z)
        print(T0H)
    
def main():
    rospy.init_node('CalcFK')
    rate = rospy.Rate(60)
    
    rot = ThetaSub()
    
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()