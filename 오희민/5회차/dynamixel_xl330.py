from dynamixel_sdk import *

# e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 57600  # XL330-M288-T의 표준 Baudrate
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095  
DXL_MOVING_STATUS_THREHOLD  = 20    # Refer to the Maximum Position Limit of product eManual
   
# 포트 열기
portHandler = PortHandler(DEVICENAME)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# 프로토콜 설정
packetHandler = PacketHandler(2.0)  # Dynamixel Protocol 2.0 사용

# XL330-M288-T 모터의 주소 설정
MOTOR_ID = 1  # XL330-M288-T의 경우 주소가 1

def user_input():
    """사용자가 계속 진행할지 확인"""
    ans = input('계속하시겠습니까? (y/n): ')
    return ans.lower() == 'y'

def move_to_goal_position(goal_position):
    """목표 위치로 이동"""
    dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, MOTOR_ID, ADDR_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("통신 에러: %s" % packetHandler.getTxRxResult(dxl_comm_result))
    else:
        print("목표 위치 설정 완료")

def main():
    """사용자 입력에 따라 목표 위치 설정"""
    while True:
        # 현재 위치 확인
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, MOTOR_ID, ADDR_PRESENT_POSITION)
        print("현재 위치: %d" % dxl_present_position)

        # 목표 각도 입력
        input_pos = int(input("목표 위치 입력: "))
        
        # 목표 위치로 이동 명령 전송
        move_to_goal_position(input_pos)

        # 사용자 입력 확인
        if not user_input():
            break

# 메인 함수 호출
main()

# 포트 닫기
portHandler.closePort()
