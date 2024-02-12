from math import *
import numpy as np

# DH Parameter
theta = np.deg2rad(np.array([0, 0, 0, 0, 0, 0]))        # rad, Z
d = np.array([89, 0, 0, 109, 95, 82])                   # length, Z
a = np.array([0, -425, -392, 0, 0, 0, 0])               # length, X
alpha = np.deg2rad(np.array([90, 0, 0, 90, -90, 0]))    # rad, X

def cal_dh_parameter(theta, d, a, alpha):
    T = [0 for _ in range(theta.size)]
    final_T = np.identity(n=4, dtype=np.int8)

    for i in range(len(T)):
        T[i] = [[cos(theta[i]), -sin(theta[i]) * cos(alpha[i]), sin(theta[i]) * sin(alpha[i]), a[i] * cos(theta[i])],
                [sin(theta[i]), cos(theta[i]) * cos(alpha[i]), -cos(theta[i]) * sin(alpha[i]), a[i] * sin(theta[i])],
                [0, sin(alpha[i]), cos(alpha[i]), d[i]],
                [0, 0, 0, 1]]
        
        final_T = final_T @ T[i]
    
    return final_T

def main():
    print(cal_dh_parameter(theta, d, a, alpha))

if __name__ == "__main__":
    main()