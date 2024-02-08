from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import matplotlib.pyplot as plt

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
        
# 로봇팔 모델 정의
chain = Chain(name='robot_arm', links=[
    OriginLink(),
    dh_to_urdf_link("joint_1", 0, 1, 0, np.pi/2, 1),  # DH 파라미터를 사용
    dh_to_urdf_link("joint_2", 0, 0, 1, 0),  # DH 파라미터를 사용
    dh_to_urdf_link("joint_3", 0, 0, 1, 0),  # DH 파라미터를 사용
])

# 엔드이펙터의 위치
target_position = [1, 1, 2]  # 여기에 원하는 x, y, z 값을 입력하세요.

# 역기구학 계산
joint_angles = chain.inverse_kinematics(target_position)

print("Calculated joint angles: ", joint_angles)

# 시각화
ax = plt.figure().add_subplot(111, projection='3d')
chain.plot(joint_angles, ax, target=target_position)
plt.show()
