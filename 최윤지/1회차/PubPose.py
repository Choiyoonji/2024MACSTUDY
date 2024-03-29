#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import Point

class PointPub:
    def __init__(self):
        self.Pose = rospy.Publisher('/pose', Point, queue_size=1)
    
def main():
    rospy.init_node('input_point')
    rate = rospy.Rate(60)
    
    point_pub = PointPub()
    
    while not rospy.is_shutdown():
        p = list(map(int,input('좌표 입력 : ').split()))
        pos = Point()
        
        pos.x = p[0]
        pos.y = p[1]
        pos.z = p[2]
        
        point_pub.Pose.publish(pos)

if __name__ == "__main__":
    main()