"""
Week 2,DH parameter / 2020112309 마태은
link 길이(로봇 팔 형상)와 각도에 따른 로봇 팔의 end effect의 위치를 DH parameter를 통해 도출함.
"""

import numpy as np
import math as mt

# Link 길이(mm)
offset = 30;
L1 = 50;
L2 = 100;
L3 = 80;

# Change degree to radian
def rad(angle):
    return angle*mt.pi/180

# About DH
class DH:
    # get angle
    def __init__(self, th1, th2, th3):
        self.th1 = th1
        self.th2 = th2
        self.th3 = th3
        # making DH table from th1, th2, th3
        self.DH_table()

    # DH table [d, theta, a, alpha]
    def DH_table(self): 
        self.DH = np.array([ 
            [offset + L1, self.th1, 0, rad(90)],
            [0, self.th2, L2, 0],
            [0, rad(90)+self.th3, -L3, 0]
        ])

    # DH transformation matrix
    def T(self, num):
        T = np.array([
            [mt.cos(self.DH[num, 1]), -mt.sin(self.DH[num, 1]), 0, self.DH[num, 2] ],
            [mt.sin(self.DH[num, 1])*mt.cos(self.DH[num, 3]), mt.cos(self.DH[num, 1])*mt.cos(self.DH[num, 3]), -mt.sin(self.DH[num, 3]), -mt.sin(self.DH[num, 3])*self.DH[num, 0]],
            [mt.sin(self.DH[num, 1])*mt.sin(self.DH[num, 3]), mt.cos(self.DH[num, 1])*mt.sin(self.DH[num, 3]), mt.cos(self.DH[num, 3]), mt.cos(self.DH[num, 3])*self.DH[num, 0]],
            [0, 0, 0, 1]
        ])
        return T
    
    # make T of end_effect
    def end_effect(self):
        T0 = self.T(0)
        T1 = self.T(1)
        T2 = self.T(2)
        T = np.dot(np.dot(T0,T1),T2)
        return T

arm = DH(0, 0, 0)
T = arm.end_effect()
print(T)