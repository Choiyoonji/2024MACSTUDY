#!/usr/bin/env python
import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 30
ADDR_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 10              # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0             # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

class MotorVelPublisher:
    def __init__(self):
        rospy.init_node('motor_vel_pub_node', anonymous=True)
        self.publisher = rospy.Publisher('set_position', SetPosition, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        rospy.loginfo('======Motor Activated======')

    def publish_callback(self):
        motor_id = int(input("Enter motor ID: "))
        position = int(input("Enter position: "))

        msg = SetPosition()
        msg.id = motor_id
        msg.position = position

        self.publisher.publish(msg)
        rospy.loginfo("Published: ID=%d, Position=%d" % (msg.id, msg.position))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, msg.id, ADDR_GOAL_POSITION, msg.position)
        # rate.sleep()
    
def main():
    ######################################################
    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL has been successfully connected")

    print("Ready to get & set Position.")
    ##################################################

    motor_vel_publisher = MotorVelPublisher()

    while not rospy.is_shutdown():
        try:
            motor_vel_publisher.publish_callback()
            motor_vel_publisher.rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()