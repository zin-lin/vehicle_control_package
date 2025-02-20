# Author : Zin Lin Htun

from dynamixel_sdk import *  # Dynamixel SDK library import
import dynamixel_sdk as dxl
import time
import rclpy
from rclpy.node import Node
from annex_msgs.msg import Con2vcu


MODE = {"SIM":1, "REAL":2}
SERVO_IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
LEG_1 = [ 12, 13, 14]
LEG_2 = [ 22, 23, 24]
LEG_3 = [ 32, 33, 34]
LEG_4 = [ 42, 43, 44]
GEAR_IDS = [14, 24, 34, 44]
OP_MODE = {"DRIVE":1, "SPIDER":2}
TORQUE_ADDR = 64
POSITION_ADDR = 116
# Set the port and baud-rate
DEVICE_NAME = '/dev/ttyUSB0'  # Modify this according to your setup
BAUDRATE = 57600  # Modify this according to your Dynamixel configuration

# Define protocol version
PROTOCOL_VERSION = 2.0

class DynamixelPublisher(Node):

    mode = MODE["REAL"]
    def __init__(self):
        super().__init__('dynamixel_publisher')


# main method
def main(args=None):
    rclpy.init(args=args)

    dynamixel_publisher = DynamixelPublisher()

    rclpy.spin(dynamixel_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dynamixel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


