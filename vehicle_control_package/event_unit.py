"""
Author: Zin Lin Htun
"""
# import ROS
from rclpy.node import Node
# import ADS-MT specifics
from annex_msgs.msg import Con2vcu
# import Sim and Practical Msgs
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
# import components
from .components.convertor import Convertor
from .constants import *
import time

class ControlEventUnit(Node):
    def __init__(self, node_name):
        # initialise
        super().__init__(node_name)
        self.servos_pos_publishers = []
        self.wheel_velocity_publishers = []
        self._sub_pub()
        self.values = VALUES # joint values
        self.wheel_values = [0.0,0.0,0.0,0.0]
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
        self.create_subscription(Con2vcu, "adsmt/manual_control",  self.control_callback, 10)
        self.create_subscription(Con2vcu, "adsmt/autonomous_control",  self.control_callback, 10)
        # create subscription for path_planning commands
        self.create_subscription(Con2vcu, "adsmt/path_planning",  self.control_callback, 10)

        # create subscription for joint states
        self.create_subscription(JointState, "adsmt/joint_states", self.joint_states_callback, 10)

        # create publications to all joints
        for joint in JOINTS:
            joint_pub = self.create_publisher(Float64,f"model/adsmt/joint/{joint}/x/cmd_pos", 10)
            self.servos_pos_publishers.append(joint_pub)

        # create publications to all wheels
        for wheel in WHEELS:
            wheel_pub = self.create_publisher(Float64,f"model/adsmt/joint/{wheel}/cmd_vel", 10)
            self.wheel_velocity_publishers.append(wheel_pub)

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

    # leaf leap begins
    def _left_leap(self):
        leg1 = Convertor.leg1_leap_for()
        leg2 = Convertor.leg2_reset()
        leg3 = Convertor.leg3_leap_for()
        leg4 = Convertor.leg4_reset()
        self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)


    # right leap begins
    def _right_leap(self):
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
        self.wheel_values = [0.0, 0.0, 0.0, 0.0]

    # turn right command
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

    # turn left command
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

    # cycle management
    def _cycle_management(self):
        # cycle even
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
            if self.stage % 2 == 1:
                self._publish_leg(LEGS['LEG-1'])  # 1 and 3 first
                self._publish_leg(LEGS['LEG-3'])

                self._publish_leg(LEGS['LEG-2'])  # 2 and 4 later
                self._publish_leg(LEGS['LEG-4'])
            else:
                self._publish_leg(LEGS['LEG-2'])  # 2 and 4 first
                self._publish_leg(LEGS['LEG-4'])

                self._publish_leg(LEGS['LEG-1'])  # 1 and 3 later
                self._publish_leg(LEGS['LEG-3'])

    # override this
    def control_callback(self, msg: Con2vcu):
        pass

    # override this
    def populate_and_publish(self):
        pass

    # override this
    def joint_states_callback(self, msg: JointState):
        pass