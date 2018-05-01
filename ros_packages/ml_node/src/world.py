import enum

class InterestPoint:
    class Type(enum.Enum):
        MACHINE = 0
        DC = 1
        DOCK = 2

    class ColorEncode(enum.Enum):
        RED = 1
        YELLOW = 2
        BLUE = 3

    def __init__(self, ip_type, position, puck_color=None):
        self.type = ip_type

        self.position = position
        self.puck = None

    def deliver_puck(self, p):
        self.puck = p

    def get_puck(self):
        if self.type == InterestPoints.Type.MACHINE:
            return self.puck
        else:
            p = self.puck
            self.puck = None

            return p
