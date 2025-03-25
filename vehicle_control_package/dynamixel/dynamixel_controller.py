"""
Zin Lin Htun
"""
import os

# import statements
from dynamixel_sdk import *  # Dynamixel SDK library import
from rclpy.node import Node
import time

LOG = "logs"
FILE = f"joints_{time.time()}.csv"
VOL_FILE = f"voltage_{time.time()}.csv"

# Dynamixel main class
class DynamixelController:
    # constructor
    def __init__(self, device_name, protocol_version, baudrate, node:Node):
        # time
        self.time = time.time()

        # set node
        self.publisher_node = node
        self.logger = self.publisher_node.get_logger()
        # initiate joints variables
        self.joints = [
           11, 12, 13,
           21, 22, 23,
           31, 32, 33,
           41, 42, 43
        ]
        self.feedback = [
                    2050, 2050, 2050,
                    2050, 2050, 2050,
                    2050, 2050, 2050,
                    2050, 2050, 2050
                    ]

        self.feedback_voltage = [
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0
                    ]
        self.wheels = [14, 24, 34, 44]
        # self.dynamixel attributes
        self.device_name = device_name
        self.protocol_version = protocol_version
        self.baudrate = baudrate
        # constants
        self.torque_address = 64
        self.velocity_address = 104
        self.position_address = 116
        self.present_position_address = 132
        self.present_input_voltage_address = 144
        self._init_dynamixel_client()
        # set up sync write
        self._initiate_sync_write()
        self.logfile = ""
        self.voltage_logfile = ""
        self._init_log()

    # initiate log
    def _init_log(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.logfile = os.path.join(current_dir, LOG, FILE)
        self.voltage_logfile = os.path.join(current_dir, LOG, VOL_FILE)
        self.logger.info("Initializing log file")
        with open(self.logfile, "a+") as file:
            file.write("11,12,13,21,22,23,31,32,33,41,42,43,time\n")

        with open(self.voltage_logfile, "a+") as file:
            file.write("11,12,13,21,22,23,31,32,33,41,42,43,time\n")

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
            self.logger.info("Succeeded to open the port")
        else:
            self.logger.info("Failed to open the port")
            exit(1)

        # Set the baud rate
        if self.portHandler.setBaudRate(self.baudrate):
            self.logger.info("Succeeded to change the baud rate")
        else:
            self.logger.info("Failed to change the baud rate")
            exit(1)

        self._set_velocity_mode()
        self._set_torque()

    # set operating mode
    def _set_velocity_mode(self ):
        for wheel in self.wheels:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, wheel, self.address_operating_mode,
                                                                           self.operating_mode)
            if dxl_comm_result != COMM_SUCCESS:
                self.logger.info(f"Failed to set velocity mode for ID {wheel}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error:
                self.logger.info(f"Error setting velocity mode for ID {wheel}: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.logger.info(f"Velocity mode enabled for ID {wheel}")

    # torque
    def _set_torque(self):
        # set torque for joints
        for joint in self.joints:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint, self.torque_address,
                                                                           1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                self.logger.info(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.logger.info("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.logger.info("Torque changed")

        # set torque for wheel
        for wheel in self.wheels:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, wheel, self.torque_address,
                                                                           1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                self.logger.info(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.logger.info("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.logger.info("Torque changed")

    # set up sync write
    def _initiate_sync_write(self):
        self.joint_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, self.position_address, 4)  # For XL330-M288T
        self.wheel_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, self.velocity_address, 4)  # For XL330-M288T
        self.joint_sync_read = GroupSyncRead(self.portHandler, self.packetHandler, self.present_position_address, 4)  # For XL330-M288T
        self.wheel_sync_read = GroupSyncRead(self.portHandler, self.packetHandler, self.present_position_address, 4)  # For XL330-M288T
        self.voltage_read = GroupSyncRead(self.portHandler, self.packetHandler, self.present_input_voltage_address, 2)  # For XL330-M288T

    # write sync param for position
    def _write_parameters_position(self, joint_id, value):
        param_goal_position = value.to_bytes(4,'little', signed=False)
        dxl_addparam_result = self.joint_sync_write.addParam(joint_id, param_goal_position)
        if not dxl_addparam_result:
            self.logger.info(f"Failed to add parameter for ID {joint_id}, - {param_goal_position}")

    # write goal position
    def write_goal_position(self, msg_positions:[]):
        for i in range(len(msg_positions)):
            self._write_parameters_position(self.joints[i], msg_positions[i])
            self.logger.info(f"Goal position for ID {msg_positions[i]}")

        # actually write stuff
        dxl_comm_result = self.joint_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info(f"Sync write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        self.logger.info("Positions set, clearing message parameters")
        # Clear the parameters after sending
        self.joint_sync_write.clearParam()

    # set feedback values
    def _get_feedback(self):

        # sync read position
        for joint_id in self.joints:
            dxl_addparam_result = self.joint_sync_read.addParam(joint_id)
            if not dxl_addparam_result:
                self.logger.info(f"Failed to add parameter for JOINT of ID {joint_id} ok ")
                # exit()

        # Perform de Operation: Sync Read
        time.sleep(0.07)
        dxl_comm_result = self.joint_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info(f"Communication failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            exit()

        # sync read voltage
        for joint_id in self.joints:
            dxl_addparam_result_v = self.voltage_read.addParam(joint_id)
            if not dxl_addparam_result_v:
                self.logger.info(f"Failed to add parameter for JOINT of ID {joint_id} ok ")
                # exit()

        # Perform de Operation: Sync Read
        time.sleep(0.07)
        dxl_comm_result_v = self.voltage_read.txRxPacket()
        if dxl_comm_result_v != COMM_SUCCESS:
            self.logger.info(f"Communication failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            exit()

        # Retrieve and log the position data for each motor
        # counter
        count = 0
        line = ""
        vol_line = ""

        # feedback for joints
        for joint_id in self.joints:
            if self.joint_sync_read.isAvailable(joint_id, self.present_position_address, 4):
                # if parameter exists then log this
                dxl_position = self.joint_sync_read.getData(joint_id, self.present_position_address, 4)
                self.feedback[count] = dxl_position
                self.logger.info(f"Dynamixel ID {joint_id} - Present Position: {dxl_position}")
                line += f"{dxl_position},"
            else:
                self.logger.info(f"Failed to get data for ID {joint_id}")
            count += 1
        for i in self.feedback:
            self.logger.info(f"Dynamixel ID {i} - Feedback: {i}")

        # feedback for voltage
        count = 0
        for joint_id in self.joints:
            if self.voltage_read.isAvailable(joint_id, self.present_input_voltage_address, 2):
                # if parameter exists then log this
                dxl_position = self.voltage_read.getData(joint_id, self.present_input_voltage_address, 2)
                self.feedback_voltage[count] = dxl_position
                self.logger.info(f"Dynamixel ID {joint_id} - Present Voltage: {dxl_position}")
                vol_line += f"{dxl_position},"
            else:
                self.logger.info(f"Failed to get data for ID {joint_id}")
            count += 1

        # log
        lapsed = time.time() - self.time
        line += f"{lapsed}\n"
        vol_line += f"{lapsed}\n"

        with open(self.logfile, "a+") as file:
            file.write(line)

        with open(self.voltage_logfile, "a+") as file:
            file.write(vol_line)

    # write sync param for velocity
    def _write_parameters_velocity(self, wheel_id, value):
        param_goal_velocity = value.to_bytes(4,'little', signed=True) # backwards
        dxl_addparam_result = self.wheel_sync_write.addParam(wheel_id, param_goal_velocity)
        if not dxl_addparam_result:
            self.logger.info(f"Failed to add parameter for ID {wheel_id}")

    # write goal velocity
    def write_goal_velocity(self, msg_values:[]):
        for i in range(len(msg_values)):
            self._write_parameters_velocity(self.wheels[i], msg_values[i])

        # actually write stuff
        dxl_comm_result = self.wheel_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.info(f"Sync write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        # Clear the parameters after sending
        self.wheel_sync_write.clearParam()

    # in range method
    def in_range(self, msg_positions:[]):
        self._get_feedback()
        for i in range(len(msg_positions)):
            if  not ((msg_positions[i] + 250) >= self.feedback[i] >= (msg_positions[i] - 250)):
                return False
        return True

    # reset
    @staticmethod
    def reset():
        return [
            2050, 1750, 1450,
            2050, 1750, 1450,
            2050, 1750, 1450,
            2050, 1750, 1450
        ]

    # close port
    def shutdown(self):
        self.portHandler.closePort()
