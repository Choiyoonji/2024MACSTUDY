from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

def dh_to_urdf_link(name, theta, d, a, alpha, z=0):
    # DH 파라미터를 URDFLink 형식으로 변환
    if z:
        return URDFLink(
            name=name,
            origin_translation=[a, 0, d],
            origin_orientation=[alpha, 0, theta],
            rotation=[0, 1, 0]
        )
    else:
        return URDFLink(
            name=name,
            origin_translation=[a, 0, d],
            origin_orientation=[alpha, 0, theta],
            rotation=[0, 0, 1]
        )
    
def calc_error(target_position, end_effector_pos):
    
    sum = 0
    for i in range(3):
        sum += (target_position[i] - end_effector_pos[i])**2

    return sum**(0.5)

def linspace(target_position, first_position):

    step_sizes = [(target_position[i] - first_position[i]) / 100 for i in range(3)]

    positions = []
    for i in range(100):
        position = [first_position[0] + i * step_sizes[0],
                    first_position[1] + i * step_sizes[1],
                    first_position[2] + i * step_sizes[2]]
        positions.append(position)

    return positions


# 로봇팔 모델 정의
chain = Chain(name='robot_arm', links=[
    OriginLink(),
    dh_to_urdf_link("joint_1", 0, 0.5, 0, np.pi/2, 1),  # DH 파라미터를 사용
    dh_to_urdf_link("joint_2", 0, 0, 0.4, 0),  # DH 파라미터를 사용
    dh_to_urdf_link("joint_3", 0, 0, 0.4, 0),  
    dh_to_urdf_link("joint_4", 0, 0, 0.3, 0),  
    #dh_to_urdf_link("joint_5", 0, 0, 0.3, 0),  
    #dh_to_urdf_link("joint_6", np.pi/2, 0, 0, np.pi/2),
])


# 엔드이펙터의 위치
first_position = [0.5, 0.5, 1]  # 여기에 원하는 x, y, z 값을 입력하세요.
# 엔드이펙터의 각도
end_effector_angles = [0, 0, 0]


# 엔드이펙터의 위치
target_position = [1, 0.3, 0.4]  # 여기에 원하는 x, y, z 값을 입력하세요.
# 엔드이펙터의 각도
end_effector_angles = [0, 0, 0]

positions = linspace(target_position, first_position)


# for position in positions:
#     # joint_angles = chain.inverse_kinematics(position, end_effector_angles)
#     # # 시각화
#     # ax = plt.figure().add_subplot(111, projection='3d')
#     # chain.plot(joint_angles, ax, target=position)
    
#     joint_angles = chain.inverse_kinematics(position, end_effector_angles)
#     # 시각화
#     ax = plt.figure().add_subplot(111, projection='3d')
#     chain.plot(joint_angles, ax, target=position)
    
#     plt.show(block=False)
#     plt.pause(0.2)  # 0.1초 동안 일시 정지
#     plt.close()  # 창을 닫음

# end_effector = chain.forward_kinematics(joint_angles)

# end_effector_pos = end_effector[0:3, 3].T

# error = calc_error(target_position, end_effector_pos)


# print("end_effector pos : ", end_effector_pos)

# print("error : ", error)

# print("Calculated joint angles: ", joint_angles)


# 시각화를 위한 초기 설정
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], 'r-')
ax.set_xlim([0,1])  # x 좌표 범위 설정
ax.set_ylim([0,1])  # y 좌표 범위 설정
ax.set_zlim([0,1])  # z 좌표 범위 설정

# 애니메이션 업데이트 함수
def update(frame):
    
    position = positions[frame]

    joint_angles = chain.inverse_kinematics(position, end_effector_angles)

    end_effector = chain.forward_kinematics(joint_angles)

    end_effector_pos = end_effector[0:3, 3].T

    error = calc_error(position, end_effector_pos)

    print("error : ", error)

    # 이전 그림 지우기
    ax.cla()
    ax.set_xlim([0,1])  # x 좌표 범위 설정
    ax.set_ylim([0,1])  # y 좌표 범위 설정
    ax.set_zlim([0,1])  # z 좌표 범위 설정
    
    # 새로운 그림 그리기
    chain.plot(joint_angles, ax, target=position)
    

    chain.plot(joint_angles, ax, target=position)
    
    return line,

# 애니메이션 생성
animation = FuncAnimation(fig, update, frames=len(positions), interval=30)

# 애니메이션 실행
plt.show()