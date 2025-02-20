"""
Zin Lin Htun
"""

# import statements
from dynamixel_sdk import *  # Dynamixel SDK library import
import dynamixel_sdk as dxl

# Dynamixel main class
class DynamixelController:
    # constructor
    def __init__(self, device_name, protocol_version, baudrate):
        # initiate joints variables
        self.joints = [11, 12, 13,
                       21, 22, 23,
                       31, 32, 33,
                       41, 42, 43]
        self.wheels = [14, 24, 34, 44]
        # self.dynamixel attributes
        self.device_name = device_name
        self.protocol_version = protocol_version
        self.baudrate = baudrate
        # constants
        self.torque_address = 64
        self.velocity_address = 104
        self.position_address = 116
        self._init_dynamixel_client()
        # set up sync write
        self._initiate_sync_write()

    # initiate dynamixel servo
    def _init_dynamixel_client(self):
        self.operating_mode = 1 # velocity
        self.address_operating_mode = 11
        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.device_name)

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(self.protocol_version)

        # Open the port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            exit(1)

        # Set the baud rate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baud rate")
        else:
            print("Failed to change the baud rate")
            exit(1)

        self._set_velocity_mode()
        self._set_torque()

    # set operating mode
    def _set_velocity_mode(self ):
        for wheel in self.wheels:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, wheel, self.address_operating_mode, self.operating_mode)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Failed to set velocity mode for ID {wheel}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error:
                print(f"Error setting velocity mode for ID {wheel}: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Velocity mode enabled for ID {wheel}")

    # torque
    def _set_torque(self):
        # set torque for joints
        for joint in self.joints:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint, self.torque_address,
                                                                           1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque changed")

        # set torque for wheel
        for wheel in self.wheels:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, wheel, self.torque_address,
                                                                           1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque changed")

    # set up sync write
    def _initiate_sync_write(self):
        self.joint_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, self.position_address, 4)  # For XL330-M288T
        self.wheel_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, self.velocity_address, 4)  # For XL330-M288T
        self.joint_sync_write = GroupSyncRead(self.portHandler, self.packetHandler, self.position_address, 4)  # For XL330-M288T
        self.wheel_sync_write = GroupSyncRead(self.portHandler, self.packetHandler, self.velocity_address, 4)  # For XL330-M288T

    # write sync param for position
    def _write_parameters_position(self, joint_id, value):
        param_goal_position = value.to_bytes(4,'little', signed=False)
        dxl_addparam_result = self.joint_sync_write.addParam(joint_id, param_goal_position)
        if not dxl_addparam_result:
            print(f"Failed to add parameter for ID {joint_id}")

    # write goal position
    def _write_goal_position(self, msg_positions:[]):
        for i in range(len(msg_positions)):
            self._write_parameters_position(self.joints[i], msg_positions[i])

        # actually write stuff
        dxl_comm_result = self.joint_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Sync write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        # Clear the parameters after sending
        self.joint_sync_write.clearParam()


    # write sync param for velocity
    def _write_parameters_velocity(self, wheel_id, value):
        param_goal_velocity = value.to_bytes(4,'little', signed=True) # backwards
        dxl_addparam_result = self.wheel_sync_write.addParam(wheel_id, param_goal_velocity)
        if not dxl_addparam_result:
            print(f"Failed to add parameter for ID {wheel_id}")


    # write goal velocity
    def _write_goal_velocity(self, msg_values:[]):
        for i in range(len(msg_values)):
            self._write_parameters_velocity(self.wheels[i], msg_values[i])

        # actually write stuff
        dxl_comm_result = self.wheel_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Sync write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        # Clear the parameters after sending
        self.wheel_sync_write.clearParam()

    # close port
    def shutdown(self):
        self.portHandler.closePort()