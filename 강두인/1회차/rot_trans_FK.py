from math import *
import numpy as np

def rot_x(theta):
    ctheta = cos(np.deg2rad(theta))
    stheta = sin(np.deg2rad(theta))
    return np.array([[1, 0, 0, 0], [0, ctheta, -stheta, 0],[0, stheta, ctheta, 0],[0, 0, 0, 1]])

def rot_y(theta):
    ctheta = cos(np.deg2rad(theta))
    stheta = sin(np.deg2rad(theta))
    return np.array([[ctheta, 0, stheta, 0], [0, 1, 0, 0], [-stheta, 0, ctheta, 0], [0, 0, 0, 1]])

def rot_z(theta):
    ctheta = cos(np.deg2rad(theta))
    stheta = sin(np.deg2rad(theta))
    return np.array([[ctheta, stheta, 0, 0], [-stheta, ctheta, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def trans(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

def cal_fk(pos_init, transformation_list):
    pos_mat = np.array([[1, 0, 0, pos_init[0]], [0, 1, 0, pos_init[1]], [0, 0, 1, pos_init[-1]], [0, 0, 0, 1]])

    for i in transformation_list:
        if i[0] == 'r':
            if i[1] == 'x':
                pos_mat = rot_x(i[2]) @ pos_mat
            elif i[1] == 'y':
                pos_mat = rot_y(i[2]) @ pos_mat
            elif i[1] == 'z':
                pos_mat = rot_z(i[2]) @ pos_mat
            elif i[1] == 'n':
                pos_mat = pos_mat @ rot_x(i[2])
            elif i[1] == 'o':
                pos_mat = pos_mat @ rot_y(i[2])
            elif i[1] == 'a':
                pos_mat = pos_mat @ rot_z(i[2])
        elif i[0] == 't':
            if i[1] == 'x':
                pos_mat = trans(i[2], 0, 0) @ pos_mat
            elif i[1] == 'y':
                pos_mat = trans(0, i[2], 0) @ pos_mat
            elif i[1] == 'z':
                pos_mat = trans(0, 0, i[2]) @ pos_mat
            elif i[1] == 'n':
                pos_mat = pos_mat @ trans(i[2], 0, 0)
            elif i[1] == 'o':
                pos_mat = pos_mat @ trans(0, i[2], 0)
            elif i[1] == 'a':
                pos_mat = pos_mat @ trans(0, 0, i[2])
    return pos_mat

def main():
    pos_init = list(map(int, input('x, y, z:').split()))
    # 예제 1
    transformation_list = [['r', 'z', 90], ['r', 'y', 90], ['t', 'x', 4], ['t', 'y', -3], ['t', 'z', 7]]
    print(cal_fk(pos_init, transformation_list))

if __name__ == "__main__":
    main()