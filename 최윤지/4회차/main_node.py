#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from math import *
import numpy as np
from geometry_msgs.msg import Point

from CalcPose import TransformMat
from CalcIK_ikpy import CalcIK


# [0]: rotation -> 'r', translation -> 't'
# [1]: x, y, z, n, o, a
# [2]: radian or meter
transformation_list = [['r', 'x', pi/2], ['t', 'a', 3], ['r','z', pi/2], ['t', 'o', 5]]

visual = 0


class PointSub:
    def __init__(self):
        self.tm = TransformMat(transformation_list)
        self.calcIK = CalcIK()
        pose = rospy.Subscriber('/pose', Point, self.callback_pose, queue_size=1)
        self.p = Point()
        
    def callback_pose(self, msg):    
        self.p.x = msg.x
        self.p.y = msg.y
        self.p.z = msg.z
        target_position = self.tm.CalcP(self.p)
        print(target_position)
        # target_position = [msg.x, msg.y, msg.z]
        theta = self.calcIK.IK(target_position, visual)
        print(theta)


def main():
    rospy.init_node('main_node')
    
    rate = rospy.Rate(60)
    
    point = PointSub()
    
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
