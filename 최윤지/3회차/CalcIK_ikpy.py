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


class CalcIK:
    def __init__(self):
        # DH parameters
        self.d0 = 1
        self.alpha0 = np.pi/2
        self.a0 = 0
        self.z0 = 1
        
        self.d1 = 0
        self.alpha1 = 0
        self.a1 = 1
        self.z1 = 0
        
        self.d2 = 0
        self.alpha2 = 0
        self.a2 = 1
        self.z2 = 0
        
        # 로봇팔 모델 정의
        self.chain = self.ChainDefinition()
        
        # 시각화
        self.ax = plt.figure().add_subplot(111, projection='3d')
    
    def ChainDefinition(self):
        return Chain(name='robot_arm', links=[
                        OriginLink(),
                        dh_to_urdf_link("joint_1", 0, self.d0, self.a0, self.alpha0, self.z0),
                        dh_to_urdf_link("joint_2", 0, self.d1, self.a1, self.alpha1, self.z1),
                        dh_to_urdf_link("joint_3", 0, self.d2, self.a2, self.alpha2, self.z2),
                    ])
        
    def IK(self, target_position, visual = 0):
        joint_angles = self.chain.inverse_kinematics(target_position)
        
        if visual:
            self.IKplot(joint_angles, target_position)
            
        return joint_angles
    
    def IKplot(self, joint_angles, target_position):
        plt.ion()
        self.chain.plot(joint_angles, self.ax, target=target_position)
        plt.show()


def main():
    Calc = CalcIK()
    while 1:
        target_position = list(map(float,input('좌표 입력 : ').split()))
        
        Calc.IK(target_position, 1)
        
        i = bool(input("continue?(0/1) : "))
        
        if not i:
            break

if __name__ == "__main__":
    main()