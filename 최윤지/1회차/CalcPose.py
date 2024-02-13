#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from math import *
import numpy as np
from numpy import linalg
from numpy import matrix as mat
from geometry_msgs.msg import Point

# [0]: rotation -> 'r', translation -> 't'
# [1]: x, y, z, n, o, a
# [2]: radian or meter
# transformation_list = [['r', 'x', pi/2], ['t', 'a', 3], ['r','z', pi/2], ['t', 'o', 5]]

class TransformMat:
    def __init__(self, transformation_list):
        self.transformation_list = transformation_list
    
    def RotX(self, theta):
        Ctheta = cos(theta)
        Stheta = sin(theta)
        return mat([[1, 0, 0, 0], [0, Ctheta, -Stheta, 0], [0, Stheta, Ctheta, 0], [0, 0, 0, 1]])
    
    def RotY(self, theta):
        Ctheta = cos(theta)
        Stheta = sin(theta)
        return mat([[Ctheta, 0, Stheta, 0], [0, 1, 0, 0], [-Stheta, 0, Ctheta, 0], [0, 0, 0, 1]])
        
    def RotZ(self, theta):
        Ctheta = cos(theta)
        Stheta = sin(theta)
        return mat([[Ctheta, -Stheta, 0, 0], [Stheta, Ctheta, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def Trans(self, x, y, z):
        return mat([[1, 0, 0, x], [0, 1, 0, y,], [0, 0, 1, z], [0, 0, 0, 1]])

    def CalcP(self, p):
        e = np.eye(3)
        e = np.vstack((e, [0, 0, 0]))
        pNew = mat([p.x, p.y, p.z, 1]).T
        pNew = np.concatenate((e, pNew), axis=1)
        for i in self.transformation_list:
            if i[0] == 'r':
                if i[1] == 'x':
                    pNew = self.RotX(i[2]) * pNew
                elif i[1] == 'y':
                    pNew = self.RotY(i[2]) * pNew
                elif i[1] == 'z':
                    pNew = self.RotZ(i[2]) * pNew
                elif i[1] == 'n':
                    pNew = pNew * self.RotX(i[2])
                elif i[1] == 'o':
                    pNew = pNew * self.RotX(i[2])
                elif i[1] == 'a':
                    pNew = pNew * self.RotX(i[2])
                else:
                    print("Check transformation_list!!!")
            elif i[0] == 't':
                if i[1] == 'x':
                    pNew = self.Trans(i[2], 0, 0) * pNew
                elif i[1] == 'y':
                    pNew = self.Trans(0, i[2], 0) * pNew
                elif i[1] == 'z':
                    pNew = self.Trans(0, 0, i[2]) * pNew
                elif i[1] == 'n':
                    pNew = pNew * self.Trans(i[2], 0, 0)
                elif i[1] == 'o':
                    pNew = pNew * self.Trans(0, i[2], 0)
                elif i[1] == 'a':
                    pNew = pNew * self.Trans(0, 0, i[2])
                else:
                    print("Check transformation_list!!!")
                    
        print(pNew)
        return pNew[:3,3].T.tolist()[0]

class PointSub:
    def __init__(self):
        transformation_list = [['r', 'x', pi/2], ['t', 'a', 3], ['r','z', pi/2], ['t', 'o', 5]]

        self.tm = TransformMat(transformation_list)
        self.pose = rospy.Subscriber('/pose', Point, self.callback_pose, queue_size=1)
        self.p = Point()
        
    def callback_pose(self, msg):
        self.p.x = msg.x
        self.p.y = msg.y
        self.p.z = msg.z
        _ = self.tm.CalcP(self.p)
        print(_)
    
def main():
    rospy.init_node('CalcPose')
    rate = rospy.Rate(60)
    
    point = PointSub()
    
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
