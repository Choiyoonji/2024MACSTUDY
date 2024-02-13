from math import *
import numpy as np

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot
import numpy as np



# 로봇팔 생성
custom_chain = Chain(name='robot_arm', links=[
    OriginLink(),
    URDFLink(
        name="origin_link",
        origin_translation=[0, 0, 0],  # 원점 위치를 설정
        origin_orientation=[0, 0, 0],  # Z 축 주변으로 회전
        rotation=[0, 0, 1],  # 회전 축을 Z 축으로 설정
    ),
    URDFLink(
      name="link1",
      origin_translation=[0, 0, 10],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="link2",
      origin_translation=[10, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="link3",
      origin_translation=[10, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    )
])


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
    # 로봇팔 원점 세팅
    pos_init = list(map(int, input('Init Pos[x, y, z] :').split()))


    # 원하는 End Effector 좌표 (x, y, z)
    target_position = list(map(int, input('End Effector[x, y, z] :').split()))

    # 역기구학
    ik_target = custom_chain.inverse_kinematics(target_position)
    _, a, b, c, _ = np.round(np.rad2deg(ik_target), 3)
    # print(np.round(np.rad2deg(ik_target), 3))

    # 로봇팔 Joint간 각도결과 출력
    print(f'First Theta: {a}, Second Theta: {b}, Third Theta: {c}')
    
    # Foward Kinematics로 End Effector 위치 확인
    transformation_list = [['r', 'a', a], ['t', 'a', 10], ['r', 'o', b], ['t', 'n', 10], ['r', 'o', c], ['t', 'n', 10]]
    print(cal_fk(pos_init, transformation_list)[:3, 3])

    # Robot_arm 시각화
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    custom_chain.plot(ik_target, ax)
    matplotlib.pyplot.show()

if __name__ == "__main__":
    main()