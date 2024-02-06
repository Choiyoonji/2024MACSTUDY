"""
Week 1,forward kinematic / 2020112309 마태은
"""

import numpy as np
import math

# change degree to radian
def degree(degree):
    return degree*(math.pi/180)

def ask():
    print('Input direction that you want to transformation(x / y / z / trans) : ')
    direc = input()
    if direc == 'x' or direc == 'y' or direc == 'z':
        print('Input value for rotation : ')
        angle = float(input())
        if direc == 'x':
            position.rot_x(degree(angle))
        if direc == 'y':
            position.rot_y(degree(angle))
        if direc == 'z':
            position.rot_z(degree(angle))  

    elif direc == "trans":
        print('Input value for translation')
        trans_list = list(map(float, input().split()))
        position.trans(trans_list)



# class for transformation
class kinematics:
    # making position matrix 
    def __init__(self, x, y, z):
        self.position = np.array([x, y, z, 1])
    
    # rotation by x-axis
    def rot_x(self, angle):
        x_rot_mat = np.array([
        [1, 0, 0, 0],
        [0, math.cos(angle), -math.sin(angle), 0],
        [0, math.sin(angle), math.cos(angle), 0],
        [0, 0, 0, 1]
        ]
        )
        self.position = np.dot(x_rot_mat, self.position)

    # rotation by y-axis
    def rot_y(self, angle):
        y_rot_mat = np.array([
        [math.cos(angle), 0, math.sin(angle), 0],
        [0, 1, 0, 0],
        [-math.sin(angle), 0, math.cos(angle), 0],
        [0, 0, 0, 1]
        ]
        )
        self.position = np.dot(y_rot_mat, self.position)

    # rotation by z-axis
    def rot_z(self, angle):
        z_rot_mat = np.array([
        [math.cos(angle), -math.sin(angle), 0, 0],
        [math.sin(angle), math.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ]
        )
        self.position = np.dot(z_rot_mat, self.position)

    # translation by inputed x, y, z
    def trans(self, trans_list):
        # list(trans_list)
        trans_list.append(0)
        trans_mat = np.array(trans_list)
        self.position = np.add(self.position, trans_mat)  

    # print current position changed from rotation and translation   
    def print(self):
        print(self.position)

    def final_print(self):
        print('Final position is ')
        print('x :', round(self.position[0],2))
        print('y :', round(self.position[1],2))
        print('z :', round(self.position[2],2))

# main code

# input init position
x, y, z= map(int, input('Please input the coordinate, x, y, z : ').split())
position = kinematics(x, y, z)

# input value for transformation
key_value = 0
while key_value==0:
    ask()
    position.print()
    print('Do you want to keep using?(Y/N) :')
    TF = input() 
    if TF == 'N':
        key_value = 1

position.final_print()