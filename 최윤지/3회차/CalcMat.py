import numpy as np
from sympy import *

theta1, theta2, theta3, l1, l2, l3 = symbols('theta1 theta2 theta3 l1 l2 l3')

T01 = Matrix([[cos(theta1), 0, sin(theta1), 0],
             [sin(theta1), 0, -cos(theta1), 0],
             [0, 1, 0, l1],
             [0, 0, 0, 1]])
T12 = Matrix([[cos(theta2), -sin(theta2), 0, l2*cos(theta2)],
             [sin(theta2), cos(theta2), 0, l2*sin(theta2)],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])
T23 = Matrix([[cos(theta3), -sin(theta3), 0, l3*cos(theta3)],
             [sin(theta3), cos(theta3), 0, l3*sin(theta3)],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

nx, ny, nz, ox, oy, oz, ax, ay, az = symbols('nx ny nz ox oy oz ax ay az')
px, py, pz = symbols('px py pz')

t = Matrix([[nx, ox, ax, px],
           [ny, oy, ay, py],
           [nz, oz, az, pz],
           [0, 0, 0, 1]])

T = simplify(T01*T12*T23)
for i in T:
    print(i)
    
print()

T1H = simplify(T12*T23)
for i in T1H:
    print(i)
    
print()

T1H2 = simplify(T01.inv()*t)
for i in T1H2:
    print(i)
    