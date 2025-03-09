"""
Author: Zin Lin Htun
name: dynamixel_helper
type: static
"""
from .dynamixel_value import DynamixelValue

class DynamixelHelper:

    @staticmethod
    def walk_forward(stage):
        positions = [
            2050, 2050, 2050,
            2050, 2050, 2050,
            2050, 2050, 2050,
            2050, 2050, 2050
        ]

        if stage == 1:
            positions = [
                2050, 1600, 1400,
                2050, 2000, 1500,
                2050, 1800, 1500,
                2050, 1800, 1900
            ]

        elif stage == 2:
            positions = [
                2050, 2000, 1500,
                2050, 1600, 1400,
                2050, 1800, 1900,
                2050, 1800, 1500
            ]

        if stage == 3:
            positions = [
                2050, 1600, 1400,
                2050, 2000, 1500,
                2050, 1800, 1500,
                2050, 1800, 1900
            ]

        elif stage == 4:
            positions = [
                2050, 2000, 1500,
                2050, 1600, 1400,
                2050, 1800, 1900,
                2050, 1800, 1500
            ]

        else:
            pass
        return positions

    # drive methods
    # static drive forward
    @staticmethod
    def drive_forward():
        velocities = [445, -445, 445, -445]
        return velocities

    # drive right
    @staticmethod
    def drive_right():
        velocities = [445, -100, 445, -100]
        return velocities

    # drive left
    @staticmethod
    def drive_left():
        velocities = [100, -445, 100, -445]
        return velocities
