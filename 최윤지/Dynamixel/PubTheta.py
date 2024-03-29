#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import Point
from manipulator.msg import *
import math
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt


AX_DXL_ID = [1, 2, 3]
XM_DXL_ID_P1 = [0]

def dh_to_urdf_link(name, theta, d, a, alpha, z=0):
    # DH 파라미터를 URDFLink 형식으로 변환
    if z == 1:
        return URDFLink(
            name=name,
            origin_translation=[a, 0, d],
            origin_orientation=[alpha, 0, theta],
            rotation=[0, 1, 0]
        )
    elif z == 2:
        return URDFLink(
            name=name,
            origin_translation=[a, 0, d],
            origin_orientation=[alpha, 0, theta],
            joint_type='fixed'
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
        self.d0 = 0.04886
        self.alpha0 = -np.pi/2
        self.a0 = 0
        self.z0 = 0
        
        self.d1 = 0
        self.alpha1 = 0
        self.a1 = 0.167
        self.z1 = 0
        
        self.d2 = 0
        self.alpha2 = 0
        self.a2 = 0.17
        self.z2 = 0
        
        self.d3 = 0
        self.alpha3 = 0
        self.a3 = 0.183
        self.z3 = 2
        
        # 로봇팔 모델 정의
        self.chain = self.ChainDefinition()
        
        # 시각화
        self.ax = plt.figure().add_subplot(111, projection='3d')
    
    def ChainDefinition(self):
        return Chain(name='robot_arm', links=[
                        OriginLink(),
                        dh_to_urdf_link("base", 0, 0.03, 0, 0, 0),
                        dh_to_urdf_link("joint_1", 0, self.d0, self.a0, self.alpha0, self.z0),
                        dh_to_urdf_link("joint_2", 0, self.d1, self.a1, self.alpha1, self.z1),
                        dh_to_urdf_link("joint_3", 0, self.d2, self.a2, self.alpha2, self.z2),
                        dh_to_urdf_link("joint_4", 0, self.d3, self.a3, self.alpha3, self.z3),
                    ],
                    active_links_mask=[False, True, True, True, True, False])
        
    def IK(self, target_position, target_orientation, visual=0):
        # joint_angles = self.chain.inverse_kinematics(target_position=target_position)
        joint_angles = self.chain.inverse_kinematics(target_position=target_position, target_orientation=[0, 0, -1], orientation_mode="X")
        
        if visual:
            self.IKplot(joint_angles, target_position)
            
        return joint_angles
    
    def IKplot(self, joint_angles, target_position):
        plt.ion()
        self.chain.plot(joint_angles, self.ax, target=target_position)
        plt.show()

    
class ThetaPub:
    def __init__(self):
        self.ThetaPublisher = rospy.Publisher('set_position', SyncSetPosition, queue_size=1)
        self.PresentPoseSub = rospy.Subscriber('present_position', SyncSetPosition, self.PoseCallback, queue_size=1) 
        
        self.PoseInfo = SyncSetPosition()
        self.flag = False
        
    def PoseCallback(self, msg):
        self.flag = True
        self.PoseInfo = msg
        
        
           


def linspace(target_position, first_position):

    step_sizes = [abs(target_position[i] - first_position[i]) / 300 for i in range(3)]

    positions = [first_position]
    for i in range(300):
        position = [first_position[0] + (i if first_position[0]<target_position[0] else -i) * step_sizes[0],
                    first_position[1] + (i if first_position[1]<target_position[1] else -i) * step_sizes[1],
                    first_position[2] + (i if first_position[2]<target_position[2] else -i) * step_sizes[2]]
        positions.append(position)

    positions.append(target_position)
    return positions


def ax_rad_to_position(rad):
    y_axis_position = 512
    add_position = round(rad*(180/math.pi)/(300/1024))
    position = y_axis_position+add_position
    if(position > 1023):
        print("overposition: ",position)
        position = 1023
    elif(position < 0) :
        position = 0
    return position


def xm_rad_to_position(rad):
    y_axis_position = 1560
    add_position = round(rad*(180/math.pi)/(360/4096))
    position = y_axis_position+add_position
    if(position > 4095):
        position = 4095
    elif(position < 0) :
        position = 0
    return position
    
    
def main():
    rospy.init_node('input_theta')
    rate = rospy.Rate(5)
    
    theta_pub = ThetaPub()
    Theta = SyncSetPosition()
    manipulator = CalcIK()
    
    Theta.ax_id = AX_DXL_ID
    Theta.xm_id_p1 = XM_DXL_ID_P1
    
    point1 = [0.2, 0.1, 0.003]
    point2 = [0.2, 0.2, 0.010]
    point3 = [0.1, 0.2, 0.010]
    point2p = [0.2, 0.2, 0.07]
    point3p = [0.12, 0.22, 0.07]
    
    orientation = [[1, 0, 0],
                   [0, -1, 0],
                   [0, 0, -1]]
    
    # point1 = [0.2, 0.1, 0.05]
    # point2 = [0.1, 0.2, 0.05]
    # point3 = [0.0, 0.3, 0.05]

    p1 = [np.linspace(point3[i],point1[i], 100).tolist() for i in range(3)]
    p2 = [np.linspace(point1[i],point2[i], 100).tolist() for i in range(3)]
    p3 = [np.linspace(point2[i],point2p[i], 10).tolist() for i in range(3)]
    p4 = [np.linspace(point2p[i],point3p[i], 100).tolist() for i in range(3)]
    p5 = [np.linspace(point3p[i],point3[i], 10).tolist() for i in range(3)]
    p6 = [np.linspace(point3[i],point2[i], 100).tolist() for i in range(3)]
    
    p = list(map(list, zip(*p1))) + list(map(list, zip(*p2))) + list(map(list, zip(*p3))) + list(map(list, zip(*p4))) + list(map(list, zip(*p5))) + list(map(list, zip(*p6)))
    # x = np.concatenate((p1[:][0], p2[:][0]))
    # y = np.concatenate((p1[:][1], p2[:][1]))
    # z = 0.05
            
    num = 0        
    t = True
    
    thetas = manipulator.IK(p[num],orientation)
    print(xm_rad_to_position(thetas[1]))
    print(thetas)    
    Theta.ax_position.append(ax_rad_to_position(thetas[2]+np.pi/2))
    Theta.ax_position.append(ax_rad_to_position(thetas[3]))
    Theta.ax_position.append(ax_rad_to_position(thetas[4]))
    Theta.xm_position_p1.append(xm_rad_to_position(thetas[1]))
    
    while not rospy.is_shutdown():
        # if theta_pub.flag:
        #     for i in range(len(Theta.ax_position)):
        #         if -7 < Theta.ax_position[i] - theta_pub.PoseInfo.ax_position[i] < 7:
                    
        #             continue
        #         else:
        #             print(Theta.ax_position[i] - theta_pub.PoseInfo.ax_position[i])
        #             t = False
        #             break
        #     for i in range(len(Theta.xm_position_p1)):
        #         if -7 < Theta.xm_position_p1[i] - theta_pub.PoseInfo.xm_position_p1[i] < 7:
        #             continue
        #         else:
        #             print('d')
                    
        #             t= False
        #             break
            
        #     if t: 
        Theta = SyncSetPosition()
        
        Theta.ax_id = AX_DXL_ID
        Theta.xm_id_p1 = XM_DXL_ID_P1

        num += 1

        if num > len(p) - 1:
            num = len(p) - 1
        # print(num)
        print(p[num])
        thetas = manipulator.IK(p[num],orientation)
        
        # Theta.ax_position = []
        # Theta.xm_position_p1 = []
        # thetas[0] = xm_rad_to_position(thetas[0])
        # for i in range(len(thetas[1:])):
        #     thetas[i+1] = ax_rad_to_position(thetas[i+1])
            
        
            # thetas[2] -= math.pi
            # if thetas[2] < -math.pi:
            #     thetas[2] += 2*math.pi
        # print(thetas[1:5])
        # for i in range(len(thetas)):
        #     if abs(thetas[i]) > np.pi:
        #         thetas[i] %= np.pi 
        
        Theta.ax_position.append(ax_rad_to_position(thetas[2]+np.pi/2))
        Theta.ax_position.append(ax_rad_to_position(thetas[3]))
        Theta.ax_position.append(ax_rad_to_position(thetas[4]))
        Theta.xm_position_p1.append(xm_rad_to_position(thetas[1]))
        # print(thetas)
            
            
        # print(t)
        theta_pub.ThetaPublisher.publish(Theta)
        # print(x)
        t = True
        
        rate.sleep()
        
        
if __name__ == "__main__":
    main()