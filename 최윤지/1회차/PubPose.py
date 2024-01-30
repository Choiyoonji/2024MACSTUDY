#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from geometry_msgs import Point

class PointPub:
    def __init__(self):
        pose = rospy.Publisher('/pose', Point, queue_size=1)
    
def main():
    rospy.init_node('input_point')
    rate = rospy.Rate(60)
    
    point = PointPub()
    
    while not rospy.is_shutdown():
        p = list(map(int(),input('좌표 입력 : ').split()))
        pos = point()
        
        pos.x = p[0]
        pos.y = p[1]
        pos.z = p[2]
        
        point.pose.publish(pos)

if __name__ == "__main__":
    main()