# Author : Zin Lin Htun

from dynamixel_sdk import *  # Dynamixel SDK library import
import dynamixel_sdk as dxl
import time
import rclpy
from rclpy.node import Node
from annex_msgs.msg import Vcu2ai


MODE = {"SIM":1, "REAL":2}
SERVO_IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
GEAR_IDS = [14, 24, 34, 44]
OP_MODE = {"DRIVE":1, "SPIDER":2}
TORQUE_ADDR = 64
POSITION_ADDR = 116

class VehicleControl(Node):

    mode = MODE["REAL"]
    op_mode = OP_MODE["SPIDER"]
    def __init__(self):
        super().__init__('vehicle_control')
        if self.mode == MODE["REAL"]:

            # Set the port and baudrate
            DEVICENAME = '/dev/ttyUSB0'  # Modify this according to your setup
            BAUDRATE = 125000  # Modify this according to your Dynamixel configuration

            # Define protocol version
            PROTOCOL_VERSION = 2.0

            # Initialize PortHandler instance
            portHandler = PortHandler(DEVICENAME)

            # Initialize PacketHandler instance
            packetHandler = PacketHandler(PROTOCOL_VERSION)

            # Open the port
            if portHandler.openPort():
                print("Succeeded to open the port")
            else:
                print("Failed to open the port")
                exit(1)

            # Set the baudrate
            if portHandler.setBaudRate(BAUDRATE):
                print("Succeeded to change the baudrate")
            else:
                print("Failed to change the baudrate")
                exit(1)

            # Enable torque for multiple motors
            for DXL_ID in SERVO_IDS:
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, TORQUE_ADDR, 1)  # Torque enable
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"TX is :: {dxl_comm_result} %s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("Error:: %s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Torque enabled")


    def leg1_2_3(self, val):

            # Write goal position
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, 116, val)  # Write goal position
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Goal position set to: {val} neutral")

    def transform (self, op_mode):
        self.op_mode = op_mode

        for DXL_ID in GEAR_IDS:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, TORQUE_ADDR, 0 if self.op_mode == OP_MODE["SPIDER"] else 1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque changed")

    def leg(self,id, val):
        # Enable torque for a single motor
        ids = [id]
        for DXL_ID in ids:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, TORQUE_ADDR, 1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque enabled")

            # Write goal position
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, POSITION_ADDR, val)  # Write goal position
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Goal position set to: {val} neutral")

    # Uncomment this section for continuous control
    # count = 1
    # while True:
    #     if count % 2 == 0:
    #         leg1_2_3(2000)
    #     else:
    #         leg1_2_3(0)
    #     count += 1

    # Set goal position for leg with ID 33 to 0
    # leg(33, 0)

    # Close port
    def end(self):
        portHandler.closePort()


# main method
def main(args=None):
    rclpy.init(args=args)

    vehicle_control = VehicleControl()

    rclpy.spin(vehicle_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


