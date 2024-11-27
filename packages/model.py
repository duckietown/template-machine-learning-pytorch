#!/usr/bin/env python3

import numpy as np
from duckietown_messages.actuators.differential_pwm import DifferentialPWM


class PyTorchModel:
    def __init__(self):
        print("initializing PyTorch model")


    def get_wheel_velocities_from_image(self, img):

        rads_left = 0.1
        rads_right = 0.1
        pwm_left = 0.4 * rads_left
        pwm_right = 0.4 * rads_right

        pwm = DifferentialPWM(left=pwm_left, right=pwm_right)
        return pwm