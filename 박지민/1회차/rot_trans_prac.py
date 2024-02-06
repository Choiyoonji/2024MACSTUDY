#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Point


def rot_x(theta):

    theta = np.deg2rad(theta)
    
    rot_x = np.array([[1,       0,      0,      0],
                      [0, np.cos(theta),-np.sin(theta), 0],
                      [0, np.sin(theta), np.cos(theta), 0],
                      [0,       0,      0,      1]])

    return rot_x

def rot_y(theta):

    theta = np.deg2rad(theta)
    
    rot_y = np.array([[np.cos(theta), 0, np.sin(theta), 0],
                      [0,       1,      0,      0],
                      [-np.sin(theta),  0, np.cos(theta), 0],
                      [0,       0,      0,      1]])

    return rot_y

def rot_z(theta):

    theta = np.deg2rad(theta)
    
    rot_z = np.array([[np.cos(theta),-np.sin(theta), 0, 0],
                      [np.sin(theta), np.cos(theta), 0, 0],
                      [0,       0,      1,      0],
                      [0,       0,      0,      1]])

    return rot_z

def trans_x(x):
    
    trans_x = np.array([[1,      0,      0,      x],
                      [0,       1,      0,      0],
                      [0,       0,      1,      0],
                      [0,       0,      0,      1]])

    return trans_x


def trans_y(y):
    
    trans_y = np.array([[1,      0,      0,      0],
                      [0,       1,      0,      y],
                      [0,       0,      1,      0],
                      [0,       0,      0,      1]])

    return trans_y


def trans_z(z):
    
    trans_z = np.array([[1,      0,      0,      0],
                      [0,       1,      0,      0],
                      [0,       0,      1,      z],
                      [0,       0,      0,      1]])

    return trans_z


def main():

    rospy.init_node('point_vector_publisher', anonymous=True)

    point_pub = rospy.Publisher('point_vec', Point, queue_size=10)

    ##ans1##

    x = 7
    y = 3
    z = 1

    p = np.array([x,y,z,1]).T
    
    fin_vec = trans_z(7)@trans_y(-3)@trans_x(4)@rot_y(90)@rot_z(90)@p

    print(fin_vec)

    
    ##ans2##
    
    T_vec = rot_z(90)@rot_x(90)@trans_z(3)@trans_y(5)

    print(T_vec.astype(float))


    point = Point(x=fin_vec[0], y=fin_vec[1], z=fin_vec[2])

    for _ in range(100):

        point_pub.publish(point)

        rospy.sleep(1)

if __name__ == "__main__":
    main()
