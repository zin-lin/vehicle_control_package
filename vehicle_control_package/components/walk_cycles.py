from .convertor import Convertor

class WalkCycles:
    def __init__(self):
        pass

    @staticmethod
    def forward_step_1(values, feedback_state):
        leg1 = Convertor.leg1_leap_for()
        leg2 = Convertor.leg2_leap_back()
        leg3 = Convertor.leg3_leap_for()
        leg4 = Convertor.leg4_leap_back()
        values = Convertor.extend_legs(leg1, leg2, leg3, leg4)
