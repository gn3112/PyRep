from pyrep.robots.mobiles.nonholonomic_base import NonHolonomicBase


<<<<<<< HEAD
class turtlebot(NonHolonomicBase):
    def __init__(self, count: int = 0, distance_from_target: float = 0):
        super().__init__(
            count, 2, distance_from_target, 'turtlebot', 4, 6, 0.035)
=======
class TurtleBot(NonHolonomicBase):
    def __init__(self, count: int = 0):
        super().__init__(count, 2, 'turtlebot')
>>>>>>> 0c964caebc4c3a0bfae31725fddadd0405a68dc7
