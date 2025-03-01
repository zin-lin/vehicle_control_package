"""
Author : Zin Lin Htun
"""

import rclpy
from rclpy.node import Node
from annex_msgs.msg import Con2vcu
from .dynamixel_controller import DynamixelController
from .dynamixel_helper import DynamixelHelper

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
OPERATION_MODE_ADDR = 11
CURRENT_POSITION_ADDR = 132
# Set the port and baud-rate
DEVICE_NAME = '/dev/ttyUSB0'  # Modify this according to your setup
BAUDRATE = 57600  # Modify this according to your Dynamixel configuration

POSITIONS = [
    2050, 2050, 2050,
    2050, 2050, 2050,
    2050, 2050, 2050,
    2050, 2050, 2050
]

# Define protocol version
PROTOCOL_VERSION = 2.0

class DynamixelPublisher(Node):
    mode = MODE["REAL"]
    def __init__(self):
        super().__init__('dynamixel_control_event_forwarding_unit')

        # initiate dynamixel controller
        self.dyn_controller = DynamixelController(DEVICE_NAME, PROTOCOL_VERSION,BAUDRATE, self)
        self.command = None
        self.steer_proportion = 1.0
        self.positions = POSITIONS
        self.velocities = [0,0,0,0]
        self.logger = self.get_logger()

        # self.attributes
        self.stage = 1

        # initiate calls
        self.start_up_pos()
        self._sub_pub()


    # start up relax-ready pose
    def start_up_pos(self):
        self.positions = [
            2050, 1800, 1200, #2000, 1500
            2050, 1800, 1200, #1600, 1400
            2050, 1600, 1500, #1800, 1900
            2050, 1600, 1500  #1800, 1500
        ]
        self.dyn_controller.write_goal_position(POSITIONS)

    # self publications and subscriptions
    def _sub_pub(self):
        self.create_subscription(Con2vcu, 'adsmt/manual_control', self.control_callback, 10)
        self.create_subscription(Con2vcu, 'adsmt/autonomous_control', self.control_callback, 10)
        # publishes every seconds
        self.timer = self.create_timer(0.2, self.publish_and_populate)

    # forward
    def _forward(self):
        self.positions = DynamixelHelper.walk_forward(self.stage)
        if self.dyn_controller.in_range(self.positions):
            if self.stage != 4:
                self.stage += 1
            else:
                self.stage = 1
        self.logger.info(f"{self.stage}")

    # drive forward
    def _drive_forward(self):
        # check if positions are reset before ever rolling forward
        if self.dyn_controller.in_range(DynamixelController.reset()):
            self.velocities = DynamixelHelper.drive_forward()

    # drive left
    def _drive_left(self):
        # check if positions are reset before ever rolling left
        if self.dyn_controller.in_range(DynamixelController.reset()):
            self.velocities = DynamixelHelper.drive_left()

    # drive right
    def _drive_right(self):
        # check if positions are reset before ever rolling right
        if self.dyn_controller.in_range(DynamixelController.reset()):
            self.velocities = DynamixelHelper.drive_right()

    # reset
    def _reset(self):
        self.positions = DynamixelController.reset()
        self.velocities = [0,0,0,0]
        self.stage = 1

    # control callback, called every time subscriptions receive a message
    def control_callback(self, msg:Con2vcu):
        self.command = msg.dir
        self.steer_proportion = msg.deg

    # publish messages
    def publish_and_populate(self):
        match self.command:
            case 1.0:
                self.velocities = [0,0,0,0]
                self._forward()
            case 2.0:
                pass
            case 3.0:
                pass
            case 5.0:
                pass
            case 6.0:
                self._reset()
                self._drive_forward()
            case 7.0:
                self._reset()
                self._drive_right()
            case 8.0:
                self._reset()
                self._drive_left()
            case _:
                self.velocities = [0,0,0,0]
                self._reset()
        self.dyn_controller.write_goal_position(self.positions)
        self.dyn_controller.write_goal_velocity(self.velocities)

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

