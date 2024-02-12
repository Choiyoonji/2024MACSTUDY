from math import *
import numpy as np

# 직교좌표계 -> 극좌표계

def cal_jacobian(x, y):
    r = sqrt(x ** 2 + y ** 2)
    theta = atan2(y, x)

    jacobian_mat = np.array([[cos(theta), -r * sin(theta)],
                            [sin(theta), r * cos(theta)]])
    
    return jacobian_mat

def main():
    x, y = map(int, input('x, y를 입력하시오: ').split())
    print(cal_jacobian(x, y))
    print()
    print('x, y 확인값')
    print(cal_jacobian(x, y) @ np.array([sqrt(2), 0.785398]))
    print('환산한 결과 값')
    print(np.linalg.inv(cal_jacobian(x, y)) @ np.array([x, y]))

if __name__ == "__main__":
    main()
