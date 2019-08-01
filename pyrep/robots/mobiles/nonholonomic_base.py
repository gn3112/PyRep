from pyrep.robots.mobiles.mobile_base import MobileBase
from pyrep.robots.configuration_paths.nonholonomic_configuration_path import (
    NonHolonomicConfigurationPath)
<<<<<<< HEAD
from pyrep.backend import utils
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.const import PYREP_SCRIPT_TYPE
from pyrep.errors import ConfigurationPathError
from typing import List
from math import sqrt


class NonHolonomicBase(MobileBase):
=======
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError
from typing import List
from math import sqrt, atan2, sin, cos


class NonHolonomicBase(MobileBase):
    """Currently only differential drive robots.
    Can be refactored to include other types of non-holonomic bases in future.
    """
>>>>>>> 0c964caebc4c3a0bfae31725fddadd0405a68dc7

    def __init__(self,
                 count: int,
                 num_wheels: int,
<<<<<<< HEAD
                 distance_from_target: float,
                 name: str,
                 max_velocity: float = 4,
                 max_velocity_rotation: float = 6,
                 max_acceleration: float = 0.035):

        super().__init__(
            count, num_wheels, distance_from_target, name,
            max_velocity, max_velocity_rotation, max_acceleration)
        self.paramP = 0.1
        self.paramO = 0.8
        self.previous_forw_back_vel = 0
        self.previous_rot_vel = 0
        self.max_acceleration = max_acceleration
        self.max_velocity = max_velocity
        self.max_vertical_rotation = max_velocity_rotation
        self.distance_from_target = distance_from_target
=======
                 name: str):

        super().__init__(count, num_wheels, name)

        self.cummulative_error = 0
        self.prev_error = 0

        # PID controller values.
        # TODO: expose to user through constructor.
        self.Kp = 1.0
        self.Ki = 0.01
        self.Kd = 0.1
        self.desired_velocity = 0.05
>>>>>>> 0c964caebc4c3a0bfae31725fddadd0405a68dc7

    def get_linear_path(self, position: List[float],
                        angle=0) -> NonHolonomicConfigurationPath:
        """Initialize linear path and check for collision along it.

        Must specify either rotation in euler or quaternions, but not both!

        :param position: The x, y position of the target.
        :param angle: The z orientation of the target (in radians).
        :raises: ConfigurationPathError if no path could be created.

        :return: A linear path in the 2d space.
        """
        position_base = self.get_position()
        angle_base = self.get_orientation()[-1]

        self.target_base.set_position(
            [position[0], position[1], self.target_z])
        self.target_base.set_orientation([0, 0, angle])
        self.intermediate_target_base.set_position(
            [position[0], position[1], self.target_z])
        self.intermediate_target_base.set_orientation([0, 0, angle])

        path = [[position_base[0], position_base[1], angle_base],
                [position[0], position[1], angle]]

        if self._check_collision_linear_path(path):
<<<<<<< HEAD
            raise ConfigurationPathError('Could not create path. An object was detected on the linear path.')
=======
            raise ConfigurationPathError(
                'Could not create path. '
                'An object was detected on the linear path.')
>>>>>>> 0c964caebc4c3a0bfae31725fddadd0405a68dc7

        return NonHolonomicConfigurationPath(self, path)

    def get_nonlinear_path(self, position: List[float],
                           angle=0,
                           boundaries=2,
                           path_pts=600,
                           ignore_collisions=False,
                           algorithm=Algos.RRTConnect
                           ) -> NonHolonomicConfigurationPath:
        """Gets a non-linear (planned) configuration path given a target pose.

        :param position: The x, y, z position of the target.
        :param angle: The z orientation of the target (in radians).
        :param boundaries: A float defining the path search in x and y direction
        [[-boundaries,boundaries],[-boundaries,boundaries]].
        :param path_pts: number of sampled points returned from the computed path
        :param ignore_collisions: If collision checking should be disabled.
        :param algorithm: Algorithm used to compute path
        :raises: ConfigurationPathError if no path could be created.

        :return: A non-linear path (x,y,angle) in the xy configuration space.
        """

        path = self._get_nonlinear_path_points(
<<<<<<< HEAD
            position, angle, boundaries, path_pts, ignore_collisions)
=======
            position, angle, boundaries, path_pts, ignore_collisions, algorithm)
>>>>>>> 0c964caebc4c3a0bfae31725fddadd0405a68dc7

        return NonHolonomicConfigurationPath(self, path)

    def get_base_actuation(self):
<<<<<<< HEAD
        """Proportional controller.
=======
        """A controller using PID.
>>>>>>> 0c964caebc4c3a0bfae31725fddadd0405a68dc7

        :return: A list with left and right joint velocity, and bool if target is reached.
        """

<<<<<<< HEAD
        handleBase = self.get_handle()
        handle_inter_target_base = self.intermediate_target_base.get_handle()
        pos_v = self.target_base.get_position(relative_to=self)
        or_v = self.target_base.get_orientation(relative_to=self)

        pos_inter = self.intermediate_target_base.get_position(
            relative_to=self)
        __, ret_floats, _, _ = utils.script_call(
            'getAngleTwoWheel@PyRep', PYREP_SCRIPT_TYPE,
            ints=[handleBase, handle_inter_target_base])

        if sqrt((pos_v[0]) ** 2 + (pos_v[1]) ** 2) < 0.01:
            return [0, 0], True

        dist_vec = sqrt(pos_inter[0] ** 2 + pos_inter[1] ** 2)
        v_des = self.paramP * dist_vec
        omega_des = self.paramO * ret_floats[0]

        v_R = v_des + self.wheel_sep * omega_des
        v_L = v_des - self.wheel_sep * omega_des

        omega_jointR = v_R / (self.wheel_size / 2)
        omega_jointL = v_L / (self.wheel_size / 2)

        return [omega_jointL, omega_jointR], False
=======
        d_x, d_y, _ = self.intermediate_target_base.get_position(
            relative_to=self)

        if sqrt((d_x) ** 2 + (d_y) ** 2) < 0.1:
            return [0., 0.], True

        alpha = atan2(d_y, d_x)
        e = atan2(sin(alpha), cos(alpha))
        e_P = e
        e_I = self.cummulative_error + e
        e_D = e - self.prev_error
        w = self.Kp * e_P + self.Ki * e_I + self.Kd * e_D
        w = atan2(sin(w), cos(w))

        self.cummulative_error = self.cummulative_error + e
        self.prev_error = e

        vr = ((2. * self.desired_velocity + w * self.wheel_distance) /
              (2. * self.wheel_radius))
        vl = ((2. * self.desired_velocity - w * self.wheel_distance) /
              (2. * self.wheel_radius))

        return [vl, vr], False
>>>>>>> 0c964caebc4c3a0bfae31725fddadd0405a68dc7
