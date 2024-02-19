#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import Point
from manipulator.msg import *
import math


AX_DXL_ID = [5]
XM_DXL_ID_P1 = [3]
XM_DXL_ID_P2 = [0]

    
class ThetaPub:
    def __init__(self):
        self.ThetaPublisher = rospy.Publisher('set_position', SyncSetPosition, queue_size=1)
    
    
def ax_deg_to_position(deg):
    y_axis_position = 512
    add_position = round(deg/(300/1024))
    position = y_axis_position+add_position
    if(position > 1023):
        print("overposition: ",position)
        position = 1023
    elif(position < 0) :
        position = 0
    return position


def xm_deg_to_position(deg):
    y_axis_position = 2048
    add_position = round(deg/(360/4096))
    position = y_axis_position+add_position
    if(position > 4095):
        position = 4095
    elif(position < 0) :
        position = 0
    return position


def ax_rad_to_position(rad):
    y_axis_position = 512
    add_position = round(rad*(180/math.pi)/(300/1024))
    position = y_axis_position+add_position
    if(position > 1023):
        print("overposition: ",position)
        position = 1023
    elif(position < 0) :
        position = 0
    return position


def xm_rad_to_position(rad):
    y_axis_position = 2048
    add_position = round(rad*(180/math.pi)/(360/4096))
    position = y_axis_position+add_position
    if(position > 4095):
        position = 4095
    elif(position < 0) :
        position = 0
    return position
    
    
def main():
    rospy.init_node('input_theta')
    rate = rospy.Rate(60)
    
    theta_pub = ThetaPub()
    Theta = SyncSetPosition()
    
    Theta.ax_id = AX_DXL_ID
    Theta.xm_id_p1 = XM_DXL_ID_P1
    Theta.xm_id_p2 = XM_DXL_ID_P2
    
    while not rospy.is_shutdown():
        t = list(map(int, input('모터 각도 입력(ax, xm1, xm2) : ').split()))
        
        Theta.ax_position.append(ax_deg_to_position(t[0]))
        Theta.xm_position_p1.append(xm_deg_to_position(t[1]))
        Theta.xm_position_p2.append(xm_deg_to_position(t[2]))
        
        theta_pub.ThetaPublisher.publish(Theta)

if __name__ == "__main__":
    main()