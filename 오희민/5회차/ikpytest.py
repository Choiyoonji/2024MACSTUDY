import numpy as np

np.float = float

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot
import numpy as np

# 로봇팔 생성
custom_chain = Chain(name='robot_arm', links=[
    OriginLink(),
    URDFLink(
        name="link1",
        translation_vector=[0, 0, 10],  # 링크의 길이와 방향을 설정
        orientation=[0, 0, 0],  # 링크의 방향을 설정
        rotation=[0, 1, 0],  # 링크의 회전을 설정
    ),
    URDFLink(
        name="link2",
        translation_vector=[10, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="link3",
        translation_vector=[10, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    )
])

# 원하는 End Effector 좌표 (x, y, z)
# 예시: np.float를 사용한 부분을 float로 변경
target_position = [3, 50, 12]


# 역기구학
ik_target = custom_chain.inverse_kinematics(target_position)
print(np.round(np.rad2deg(ik_target), 3))

# Robot_arm 시각화
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
custom_chain.plot(ik_target, ax)
matplotlib.pyplot.show()
