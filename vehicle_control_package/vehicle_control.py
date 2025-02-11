"""
Author: Zin Lin Htun
"""
import time
import rclpy
from rclpy.node import Node
from annex_msgs.msg import Con2vcu, Ai2vcu
from os import environ as env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from .components.convertor import Convertor

# CONSTANTS

JOINTS = [
        'joint_1', 'joint_1_1', 'joint_1_1_1',
        'joint_2', 'joint_2_1', 'joint_2_1_1',
        'joint_3', 'joint_3_1', 'joint_3_1_1',
        'joint_4', 'joint_4_1', 'joint_4_1_1'
        ]

VALUES = [0.0,0.0,0.0,
          0.0,0.0,0.0,
          0.0,0.0,0.0,
          0.0,0.0,0.0]

LEGS = {'LEG-1':0, 'LEG-2':3, 'LEG-3':6, 'LEG-4':9}

# main class
class VehicleControlEventUnit(Node):
    def __init__(self):
        # initialise
        super().__init__('vehicle_control_event_unit')
        self.servos_pos_publishers = []
        self._sub_pub()
        self.values = VALUES
        self.command = None
        self.stage = 1
        self.cycle = 1

        # initialise feedback
        self.feedback_state = []

        # initialise logger
        self.logger = self.get_logger()

        # initialise populate timer
        self.publisher_timer = self.create_timer(0.1, self.populate_and_publish)

        # reset to starting pose
        self._start_up_pos()

    # subscription and publishing
    def _sub_pub(self):
        # create subscription for control commands
        self.create_subscription(Con2vcu, "control",  self.control_callback, 10)

        # create subscription for joint states
        self.create_subscription(JointState, "adsmt/joint_states", self.joint_states_callback, 10)

        # create publications to all joints
        for joint in JOINTS:
            joint_pub = self.create_publisher(Float64,f"model/adsmt/joint/{joint}/x/cmd_pos", 10)
            self.servos_pos_publishers.append(joint_pub)

    # start up position publisher
    def _start_up_pos(self):
        self.values = []
        self.values = Convertor.start_up_pos()

    # for each index range
    def _publish_range_index (self, index, stop,step):
        for i in range(index, stop, step):
            msg = Float64()
            msg.data = self.values[i]
            self.servos_pos_publishers[i].publish(msg)
            self.logger.info(f"published - {JOINTS[i]} -  {msg.data}")

    def _publish_leg(self, index):
        for i in range(index,index+3,1):
            msg = Float64()
            msg.data = self.values[i]
            self.servos_pos_publishers[i].publish(msg)
            self.logger.info(f"published - {JOINTS[i]} -  {msg.data}")

    # right leap
    def _for_leap(self):
        leg1 = Convertor.leg1_leap_for()
        leg2 = Convertor.leg2_reset()
        leg3 = Convertor.leg3_leap_for()
        leg4 = Convertor.leg4_reset()
        self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)

    # left leap
    def _back_leap(self):
        leg1 = Convertor.leg1_reset()
        leg2 = Convertor.leg2_leap_for()
        leg3 = Convertor.leg3_reset()
        leg4 = Convertor.leg4_leap_for()
        self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)

    # all four reset
    def _reset_walk(self):
        leg1 = Convertor.leg1_reset()
        leg2 = Convertor.leg2_reset()
        leg3 = Convertor.leg3_reset()
        leg4 = Convertor.leg4_reset()
        self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)

    # reset all states
    def _reset_state(self):
        self.stage = 1
        self.command = None

    # turn right
    def _turn_right(self):
        if self.stage == 1:
            # first turn
            self.values[LEGS['LEG-2']] = Convertor.joint_2_turn()
            self.values[LEGS['LEG-4']] = Convertor.joint_4_turn()
            leg1 = Convertor.leg1_leap_for()
            leg3 = Convertor.leg3_leap_for()
            leg2 = Convertor.leg_rest(self.values, LEGS['LEG-2'])
            leg4 = Convertor.leg_rest(self.values, LEGS['LEG-4'])
            self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)

            # check if position reached
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 2
        elif self.stage == 2:
            # reset
            self._reset_walk()
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 1
                time.sleep(0.100)

    def _turn_left(self):
        if self.stage == 1:
            # first turn
            self.values[LEGS['LEG-1']] = Convertor.joint_1_turn()
            self.values[LEGS['LEG-3']] = Convertor.joint_3_turn()
            leg2 = Convertor.leg2_leap_for()
            leg4 = Convertor.leg4_leap_for()
            leg1 = Convertor.leg_rest(self.values, LEGS['LEG-1'])
            leg3 = Convertor.leg_rest(self.values, LEGS['LEG-3'])
            self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)

            # check if position reached
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 2
        elif self.stage == 2:
            # reset
            self._reset_walk()
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 1
                time.sleep(0.100)

                # move forward
    def _forward_walk(self):
        self.logger.info("w called")

        if not self.stage:
            self.command = "w"
            self.stage = 1

        if self.stage == 1:
            # first half stage 2
            self.logger.info("getting to stage 1")
            if self.cycle%2 == 0:
                self._for_leap()
            else:
                self._back_leap()

            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 2


        if self.stage == 2:
            # second half stage 3
            self.logger.info("getting to stage 3")
            # stage 3
            if self.cycle%2 == 0:
                self._back_leap()
            else:
                self._for_leap()

            if Convertor.in_range(self.values, self.feedback_state):
                # resets
                self.stage = 3

        # reset
        if self.stage == 3:
            self.logger.info("getting to stage 2")
            self._reset_walk()
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 4

        if self.stage == 4:
            # second half stage 3
            self.logger.info("getting to stage 3")
            # stage 3
            if self.cycle%2 == 0:
                self._back_leap()
            else:
                self._for_leap()
            if Convertor.in_range(self.values, self.feedback_state):
                # resets
                self.stage = 5

        if self.stage == 5:
            # first half stage 2
            self.logger.info("getting to stage 1")
            if self.cycle % 2 == 0:
                self._for_leap()
            else:
                self._back_leap()

            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 6

        if self.stage == 6:
            self.logger.info("getting to stage 4")
            self._reset_walk()
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 1
                self.cycle += 1
                time.sleep(0.100)

    # side turn walk
    def _turn_walk(self, dir_walk):
        if dir_walk:
            self._turn_right()
        else:
            self._turn_left()

    # subscription- joint_states callback
    def joint_states_callback(self, msg:JointState):
        self.feedback_state = msg.position

    # subscription- control command callback
    def control_callback(self, msg:Con2vcu):
        cmd = msg.dir
        self.stage = 1
        if cmd == 1.0:
            self.command = "w"
            self.logger.info('w')
        elif cmd == 2.0:
            self.command = "a"
            self.logger.info('a')
        elif cmd == 3.0:
            self.command = "d"
            self.logger.info('d')
        elif cmd == 4.0:
            self.command = "s"
            self.logger.info('s')
        else:
            self.command = None
            self.logger.info('unknown/stop command: stopping')


    # populate and publish the message
    def populate_and_publish(self):
        # match the command
        match self.command:
            case "w":
                self._forward_walk()
            case "a":
                self._turn_walk(False)
            case "d":
                self._turn_walk(True)
            case "s":
                pass

            case _:
                self._reset_walk()
                self._reset_state()
                self.logger.error('no command, just publishing')

        # first joints
        if self.cycle % 2 == 0:

            if self.stage % 2 == 1:
                self._publish_leg(LEGS['LEG-2'])  # 2 and 4 first
                self._publish_leg(LEGS['LEG-4'])

                self._publish_leg(LEGS['LEG-1'])  # 1 and 3 later
                self._publish_leg(LEGS['LEG-3'])
            else:
                self._publish_leg(LEGS['LEG-1'])  # 1 and 3 first
                self._publish_leg(LEGS['LEG-3'])

                self._publish_leg(LEGS['LEG-2'])  # 2 and 4 later
                self._publish_leg(LEGS['LEG-4'])



        else:
            # cycle odd
            if self.stage % 2== 1:
                self._publish_leg(LEGS['LEG-1']) # 1 and 3 first
                self._publish_leg(LEGS['LEG-3'])

                self._publish_leg(LEGS['LEG-2']) # 2 and 4 later
                self._publish_leg(LEGS['LEG-4'])
            else:
                self._publish_leg(LEGS['LEG-2'])  # 2 and 4 first
                self._publish_leg(LEGS['LEG-4'])

                self._publish_leg(LEGS['LEG-1']) # 1 and 3 later
                self._publish_leg(LEGS['LEG-3'])


# main method
def main(args=None):
    rclpy.init(args=args)
    # get vcu node
    vcu_node = VehicleControlEventUnit()
    # spin it
    rclpy.spin(vcu_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vcu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()