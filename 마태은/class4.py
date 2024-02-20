import numpy as np
import math as mt

# Link 길이(mm)
offset = 30
L1 = 50
L2 = 100
L3 = 80

# Change degree to radian
def rad(angle):
    return angle * mt.pi / 180

def deg(angle):
    return angle * 180 / mt.pi
# About DH
class DH:
    # get angle
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        # Solve inverse kinematics
        self.inverse_kinematics()

    # Solve inverse kinematics
    def inverse_kinematics(self):
        x = self.x - offset
        y = self.y
        z = self.z
        th1 = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        d = z
        a = r - L1
        D = (a**2 + d**2 - L2**2 - L3**2) / (2 * L2 * L3)
        th3 = np.arctan2(-np.sqrt(1 - D**2), D)
        th2 = np.arctan2(d, a) - np.arctan2(L3 * np.sin(th3), L2 + L3 * np.cos(th3))
        # Convert to degree
        self.th1 = deg(th1)
        self.th2 = deg(th2)
        self.th3 = deg(th3)

    # Return angles
    def get_angles(self):
        return self.th1, self.th2, self.th3

# Example usage
x = 0
y = 0
z = 0
x, y, z = map(int,input('input : ').split())
arm = DH(x, y, z)
th1, th2, th3 = arm.get_angles()
print("각도:", th1, th2, th3)