"""
1. Forward Kinematics
로봇팔의 회전행렬(rotation matrix)와 변환행렬(translation matrix)를
함수로 구현하는 코드
"""

import numpy as np
import math

PI = math.pi


def rot_x(point, theta):  # rotate about x-axis, theta is rad
    rot_mat = np.array([[1, 0, 0, 0], [0, math.cos(theta), -math.sin(theta), 0],
                       [0, math.sin(theta), math.cos(theta), 0], [0, 0, 0, 1]])

    return np.dot(rot_mat, point)


def rot_y(point, theta):  # rotate about y-axis, theta is rad
    rot_mat = np.array([[math.cos(theta), 0, math.sin(theta), 0], [
                       0, 1, 0, 0], [-math.sin(theta), 0, math.cos(theta), 0], [0, 0, 0, 1]])

    return np.dot(rot_mat, point)


def rot_z(point, theta):  # rotate about z-axis, theta is rad
    rot_mat = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                       [math.sin(theta), math.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    return np.dot(rot_mat, point)


def trans(point, tx=0, ty=0, tz=0):  # translation code
    trans_mat = np.array([[1, 0, 0, tx], [0, 1, 0, ty],
                         [0, 0, 1, tz], [0, 0, 0, 1]])

    return np.dot(trans_mat, point)


# main
print("Please Input the coordinate:")
point = np.array(list(map(float, input().split())))

print("Please Input the rotation angle(just number):")
angle = float(input())

print("Is it radian?(T/F)")
radian = input()

if radian == 'f' or radian == 'F':
    angle = np.deg2rad(angle)

point = np.append(point, 1)  # homogeneous coordinate
print(point)  # original point

point = rot_z(point, angle)
point = rot_y(point, angle)
point = trans(point, 4, -3, 7)
print(point)  # point after transforming
