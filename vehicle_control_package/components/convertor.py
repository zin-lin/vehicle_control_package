"""
Author: Zin Lin Htun
"""

class Convertor:
    def __init__(self):
        # static class
        pass

    # reset methods
    @staticmethod
    def leg1_reset():
        return [0.0, 0.55, -1.45]

    @staticmethod
    def leg2_reset():
        return [0.0, 0.55, -1.45]

    @staticmethod
    def leg4_reset():
        return [0.0, -0.55, 1.45]

    @staticmethod
    def leg3_reset():
        return [0.0, -0.55, 1.45]

    # rest methods
    @staticmethod
    def leg_rest(values:list[float], index:int):
        leg = []
        for i in range(index, index + 3, 1):
            leg.append(values[i])
        return leg

    # draw_back
    @staticmethod
    def leg1_leap_back():
        return [0.0, 0.75, -1.55]

    @staticmethod
    def leg2_leap_back():
        return [0.0, -0.75, 1.55]

    @staticmethod
    def leg4_leap_back():
        return [0.0, 0.75, -1.55]

    @staticmethod
    def leg3_leap_back():
        return [0.0, -0.75, 1.55]

    # leap forward
    @staticmethod
    def leg1_leap_for():
        return [.05, 0.05, -1.00]

    @staticmethod
    def leg2_leap_for():
        return [-.05, 0.05, -1.00]

    @staticmethod
    def leg3_leap_for():
        return [.05, -0.05, 1.0]

    @staticmethod
    def leg4_leap_for():
        return [-0.05, -0.05, 1.00]

    @staticmethod
    def joint_1_turn():
        return -0.23

    @staticmethod
    def joint_2_turn():
        return 0.23

    @staticmethod
    def joint_3_turn():
        return -0.1

    @staticmethod
    def joint_4_turn():
        return 0.1

    # extend list
    @staticmethod
    def extend_legs(leg1, leg2, leg3, leg4):
        array = []
        array.extend(leg1)
        array.extend(leg2)
        array.extend(leg3)
        array.extend(leg4)
        return array

    # compare two lists, see if values are in acceptable range
    @staticmethod
    def in_range(list1, list2):
        for i in range(len(list1)):
            if  not ((list2[i] + 0.005) >= list1[i] >= (list2[i] - 0.005)):
                return False
        return True

    # get start up position
    @staticmethod
    def start_up_pos():
        leg1 = Convertor.leg1_reset()
        leg2 = Convertor.leg2_reset()
        leg3 = Convertor.leg3_reset()
        leg4 = Convertor.leg4_reset()
        values = []
        values.extend(Convertor.extend_legs(leg1, leg2, leg3, leg4))
        return values