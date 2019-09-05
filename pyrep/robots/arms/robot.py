from pyrep.robots.arms.arm import Arm

class robot(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'robot', 4)
