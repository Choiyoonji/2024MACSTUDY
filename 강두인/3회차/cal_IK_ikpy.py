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

# 원하는 End Effector 좌표 (x, y, z)
target_position = [12, 12, 12]

# 역기구학
ik_target = custom_chain.inverse_kinematics(target_position)
_, a, b, c, _ = np.round(np.rad2deg(ik_target), 3)
# print(np.round(np.rad2deg(ik_target), 3))

# 결과 출력
print(custom_chain.forward_kinematics([0] * 5)[:3, 3])
print(f'First Theta: {a}, Second Theta: {b}, Third Theta: {c}')

# Robot_arm 시각화
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
custom_chain.plot(ik_target, ax)
matplotlib.pyplot.show()