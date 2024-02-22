#!/usr/bin/env python3

import os
import rospy
from robot_arm.msg import *

def main():
    
    rospy.init_node('angle_pub_node')
    pub_angle = rospy.Publisher('set_position', jointset, queue_size = 1)

    while(True):
        
        js = jointset()

        js.id = int(input("input motor id : "))

        js.position = float(input("input motor position : "))

        pub_angle.publish(js)

        a = input("u wanna terminate [y/n] : ")

        if a == "y":
            break


if __name__ == '__main__':
    main()

