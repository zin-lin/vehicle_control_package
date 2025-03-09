"""
Author: Zin Lin Htun
"""
# import py libs
# import ROS
import rclpy
from .event_unit import *


# main class
class VehicleControlEventUnit(ControlEventUnit):
    def __init__(self):
        super().__init__('vehicle_control_event_forwarding_unit')
        self.deg = 1.0 # proportion to one

    # def forward jump
    def _forward_jump(self):
        self.logger.info("jf called")
        if self.stage == 1:
            leg1 = Convertor.leg1_reset()
            leg2 = Convertor.leg2_reset()
            leg3 = Convertor.leg3_leap_for()
            leg4 = Convertor.leg4_leap_for()
            self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 2

        if self.stage == 2:
            leg1 = Convertor.leg1_jump_for()
            leg2 = Convertor.leg2_jump_for()
            leg3 = Convertor.leg3_leap_back()
            leg4 = Convertor.leg4_leap_back()
            self.values = Convertor.extend_legs(leg1, leg2, leg3, leg4)
            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 3

        if self.stage == 3:
            self._reset_walk()
            self._reset_state()


    # drive commands
    # drive forward
    def _drive_forward(self):
        self.logger.info("df called")
        self.wheel_values = None
        self.wheel_values = [15.0, 15.0, -15.0, -15.0]

    # drive right
    def _drive_right(self, k_rad=0.5, i_vel=15.0 ):
        self.logger.info("dr called")
        self.wheel_values = None
        value = i_vel # initial turn velocity
        self.wheel_values = [value, (value*k_rad), -value, -(value*k_rad)]

    # drive left
    def _drive_left(self, k_rad=0.5, i_vel=15.0):
        self.logger.info("dl called")
        self.wheel_values = None
        value = i_vel # initial turn velocity
        self.wheel_values = [(value*k_rad), value, -(value*k_rad), -value]

    # move forward
    def _forward_walk(self):
        self.logger.info("w called")

        # stage setter if not available
        if not self.stage:
            self.command = "w"
            self.stage = 1

        # stage 1
        if self.stage == 1:
            # first half stage 2
            self.logger.info("getting to stage 1")
            self._left_leap()

            if Convertor.in_range(self.values, self.feedback_state):
                self.stage = 2

        # stage 2
        if self.stage == 2:
            # second half stage 2
            self.logger.info("getting to stage 2")
            # stage 3
            self._right_leap()

            if Convertor.in_range(self.values, self.feedback_state):
                # resets
                self.stage = 1

        # # stage 3
        # if self.stage == 3:
        #     self.logger.info("getting to stage 3")
        #     self._reset_walk()
        #     if Convertor.in_range(self.values, self.feedback_state):
        #         self.stage = 1
        #         self.cycle+=1


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
        # if mode changes reset states
        if self.command != msg.dir:
            self._reset_state()

        self.command = msg.dir
        if msg.deg == 0:
            self.deg = 0.5
        else:
            self.deg = msg.deg

    # populate and publish the message use to publish commands to either sim or dynamixel servos
    def populate_and_publish(self):
        # match the command
        match self.command:
            case 1.0:
                self._forward_walk()
            case 2.0:
                self._turn_walk(False)
            case 3.0:
                self._turn_walk(True)
            case 4.0:
                pass
            case 5.0:
                self._forward_jump()
            case 6.0:
                self._reset_walk() # important to have an unbiased angular drive
                self._drive_forward()
            case 7.0:
                self._reset_walk() # important to have an unbiased angular drive
                self._drive_right(self.deg)
            case 8.0:
                self._reset_walk() # important to have an unbiased angular drive
                self._drive_left(self.deg)
            # default
            case _:
                self._reset_walk()
                self._reset_state()
                self.logger.error('no command, just publishing')

        """
            Cycles are set so that there won't be biases on each side
            This is important to limit skidding 
            This wouldn't be an issue with dynamixel servos as one can use sync read and sync write. 
            Everything will be published immediately. 
        """
        self._cycle_management()
        # wheel publishers
        for i in range(len(self.wheel_velocity_publishers)):
            msg = Float64()
            msg.data = self.wheel_values[i]
            self.wheel_velocity_publishers[i].publish(msg)
            self.logger.info(f"published - {WHEELS[i]} -  {msg.data}")


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