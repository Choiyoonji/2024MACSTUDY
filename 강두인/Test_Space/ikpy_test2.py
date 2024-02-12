from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot
import numpy as np



# 로봇팔 생성
left_arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="shoulder",
      origin_translation=[-10, 0, 5],
      origin_orientation=[0, 1.57, 1.57],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="elbow",
      origin_translation=[25, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="wrist",
      origin_translation=[22, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
])

# 원하는 End Effector 좌표 (x, y, z)
target_position = [12, 12, 12]

# 역기구학
ik_target = left_arm_chain.inverse_kinematics(target_position)
print(ik_target)

# Robot_arm 시각화
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
left_arm_chain.plot(ik_target, ax)
matplotlib.pyplot.show()