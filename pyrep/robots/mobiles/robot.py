from pyrep.robots.mobiles.holonomic_base import HolonomicBase

class robot(HolonomicBase):
    def __init__(self, count: int = 0, distance_from_target: int = 0.25):
        super().__init__(
            count, 4, distance_from_target, 'robot', 4, 6, 0.035)
