########################################################################################
# meArm.py 
# A motion control library for Phenoptix meArm using Adafruit PCA9685 servo driver
########################################################################################
#
# Original: York Hack Space May 2014
#
# Updates by Urs Utzinger
#
# Summer 2025
# - Converted to adafruit_motor, adafruit_pca9685
#   this no longer uses a gain as motors have consistent relation ship between pulsewidth
#   and angle. We only need to calibrate offset due to assembly of motore axle onto system
# - Refactored with ChatGPT 2025
# Spring 2025
# - partial gripper open
# - returns whether the move completed successfully
# - keeps current x,y,z,gripper as property
# - does not move beyond max and min angle as defined by meArm desing mechanics
#
########################################################################################
#
# In kinmeatics.py the convetion for angles is:
#   base view from top, clockwise is positive and zero is arm pointing straight forward
#   shoulder-elbow horizontal (forward) is zero and straight up is 90
#   elbow-wrist horizontal is zero and moving gripper downwards is negative
#
# To match these angle convention with motor angles we observe:
#
# Base     
#   base motor  90 is straight forward with base angle 0, 
#   base motor   0 is base angle 90, 
#   base motor 180 is base angle -90  
# Shoulder 
#   shoulder motor  90 is straight up with shoulder angle 90, 
#   shoulder motor 180 is shoulder angle 0, 
#   shoulder motor  30 is shoulder angle 150
# Elbow    
#   elbow motor  90 is horizontal with elbow angle 0, 
#   elbow motor 120 is elbow angle  30, 
#   elbow motor  10 is elbow angle -80
# Gripper  
#   gripper motor 125 is closed, 
#   gripper motor 0 is open
#
# For maximum movement without collisions in the mechanics we define the range of the angles as:
#     base  -90 ...  90
#  soulder    0 ... 150
#    elbow -180 ...  30
#
# The offset is  = (zero - 90) which we add to the motor angle before we move the motor
#
# motorBase     =  90. - deg + zero - 90.
# motorShoulder = 180. - deg + zero - 90.
# motorElbow    =  90. + deg + zero - 90.
# motorGripper  = deg + (zero - max) 
#
########################################################################################

# ##############################################################################
# 
# The MIT License (MIT)
# 
# Copyright (c) York Hack Space May 2014, 2025 Urs Utzinger
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ##############################################################################


import os
import json
import time
import logging
from typing import Tuple, Optional

import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

import math
import kinematics

# Constants for unit conversion
default_logger = logging.getLogger(__name__)
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
DEFAULT_CONFIG = "mearm_config.json"

def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a float between min_val and max_val."""
    return max(min_val, min(value, max_val))

class meArm:
    """
    High-level control for the meArm robot arm.
    """
    def __init__(
        self,
        i2c=None,
        address: int = 0x40,
        config_file: str = DEFAULT_CONFIG,
        logger: logging.Logger = default_logger,
    ):
        self.logger = logger
        self.config_file = config_file
        self._load_config()

        # Initialize PCA9685
        self.i2c = i2c or board.I2C()
        self.pca = PCA9685(self.i2c, address=address)
        self.pca.frequency = 50

        # Setup servos with calibration and limits
        self.servos = {
            'base': self._setup_servo(0, self.calib['base']),
            'shoulder': self._setup_servo(1, self.calib['shoulder']),
            'elbow': self._setup_servo(14, self.calib['elbow']),
            'gripper': self._setup_servo(15, self.calib['gripper']),
        }

        # Track current state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.finger = 0.0

        # Home wrist and gripper
        self.open_gripper()
        self.move_to(0.0, 150.0, 100.0)

    def _load_config(self) -> None:
        """Load calibration zeros and limits from JSON config."""
        if not os.path.exists(self.config_file):
            raise FileNotFoundError(f"Config file not found: {self.config_file}")
        with open(self.config_file) as f:
            data = json.load(f)
        # Expect keys: base, shoulder, elbow, gripper each with dict: zero, min_deg, max_deg
        self.calib = data
        self.logger.debug("Loaded calibration: %s", self.calib)

    def _setup_servo(self, channel: int, info: dict) -> servo.Servo:
        """
        Create and configure a servo on a given PCA channel.
        `info` should contain 'zero', 'min_deg', 'max_deg'.
        """
        srv = servo.Servo(
            self.pca.channels[channel], min_pulse=500, max_pulse=2500
        )
        srv._zero = info['zero']
        srv._min  = info['min_deg']
        srv._max  = info['max_deg']
        return srv

    def _angle_limits(self, joint: str, angle_rad: float) -> float:
        """Clamp a joint angle (rad) to its configured limits."""
        deg = angle_rad * RAD2DEG
        deg = clamp(deg, self.calib[joint]['min_deg'], self.calib[joint]['max_deg'])
        return deg * DEG2RAD

    def _angle_to_servo(self, joint: str, angle_rad: float) -> float:
        """
        Convert a joint angle (rad) to a servo command (degrees),
        applying the zero offset and per-joint direction.
        
        motorBase     =  90. - deg + zero - 90.
        motorShoulder = 180. - deg + zero - 90.
        motorElbow    =  90. + deg + zero - 90.
        motorGripper  = deg + zero - max

        """
        deg = angle_rad * RAD2DEG
        zero = self.calib[joint]['zero']
        if joint == 'base':
            cmd = - deg + zero
        elif joint == 'shoulder':
            cmd = 90. - deg + zero
        elif joint == 'elbow':
            cmd =  + deg + zero
        elif joint == 'gripper':
            cmd = deg + zero - self.calib[joint]['max_deg']

        # Clamp final command to servo limits
        return clamp(cmd, 0, 180)

    def _servo_to_angle(self, joint: str, cmd: float) -> float:
        """
        Convert a servo command (degrees) to joint angle (rad),
        removing the zero offset and per-joint direction.
        
        motorBase     =  90. - deg + zero - 90.
        motorShoulder = 180. - deg + zero - 90.
        motorElbow    =  90. + deg + zero - 90.
        motorGripper  = deg + zero - max
        
        """
        zero = self.calib[joint]['zero']
        if joint == 'base':
            deg = zero - cmd
        elif joint == 'shoulder':
            deg = 90 + zero - cmd
        elif joint == 'elbow':
            deg = cmd - zero
        elif joint == 'gripper':
            deg = cmd - zero + self.calib[joint]['max_deg']

        return deg * DEG2RAD

    def move_to(self, x: float, y: float, z: float) -> bool:
        """
        Move end effector to an (x,y,z) position in a single step.
        Returns True on success, False if out of reach or errors.
        """
        self.logger.info("Attempting to move to (%.1f, %.1f, %.1f)", x, y, z)

        # Absolute Max Positions
        # x 220 .. -220
        # y 217 ..    7
        # z 120 ..  -79
        x = clamp(x, -220., 220.)
        y = clamp(y,    5., 220.)
        z = clamp(z,  -50., 120.)

        # compute angles
        sol = kinematics.inverse_kinematics(x, y, z)
        if sol is None:
            self.logger.warning("Position out of inverse kinematics range: %s", (x, y, z))
            return False
        theta0, theta1, theta2 = sol
        # clamp to joint limits
        theta0 = self._angle_limits('base', theta0)
        theta1 = self._angle_limits('shoulder', theta1)
        theta2 = self._angle_limits('elbow', theta2)
        # convert to motor angles
        motor_base     = self._angle_to_servo('base', theta0)
        motor_shoulder = self._angle_to_servo('shoulder', theta1)
        motor_elbow    = self._angle_to_servo('elbow', theta2)
        # send to servos
        self.servos['base'].angle     = motor_base
        self.servos['shoulder'].angle = motor_shoulder
        self.servos['elbow'].angle    = motor_elbow 

        # update state taking into account clipping that migth have occured
        theta0 = self._servo_to_angle('base', motor_base)
        theta1 = self._servo_to_angle('shoulder', motor_shoulder)
        theta2 = self._servo_to_angle('elbow', motor_elbow)
        x,y,z = kinematics.forward_kinematics(theta0, theta1, theta2)
        self.x, self.y, self.z = x, y, z
        self.logger.info("Moved to (%.1f, %.1f, %.1f)", x, y, z)
        return True

    def move_linear(self, x: float, y: float, z: float, step: float = 10.0, delay: float = 0.05) -> bool:
        """
        Move in straight-line increments from current position to (x,y,z).
        """
        x0, y0, z0 = self.x, self.y, self.z
        dist = kinematics.distance_3d((x0, y0, z0), (x, y, z))
        if dist == 0:
            return True
        n = int(math.ceil(dist / step))
        for i in range(1, n + 1):
            xi = x0 + (x - x0) * i / n
            yi = y0 + (y - y0) * i / n
            zi = z0 + (z - z0) * i / n
            if not self.move_to(xi, yi, zi):
                return False
            time.sleep(delay)
        return True

    def open_gripper(self) -> None:
        """Fully open the gripper."""
        # Use full-open joint angle (radians) then map to servo
        angle_rad = self.calib['gripper']['min_deg'] * DEG2RAD
        self.servos['gripper'].angle = self._angle_to_servo('gripper', angle_rad)
        self.finger = 100.0
        self.logger.info("Gripper opened")

    def close_gripper(self) -> None:
        """Fully close the gripper."""
        angle_rad = self.calib['gripper']['max_deg'] * DEG2RAD
        self.servos['gripper'].angle = self._angle_to_servo('gripper', angle_rad)
        self.finger = 0.0
        self.logger.info("Gripper closed")

    def partial_grip(self, percent: float) -> None:
        """
        Set gripper opening to a percentage (0=closed, 100=open).
        """
        pct = clamp(percent, 0.0, 100.0)
        info = self.calib['gripper']
        # Compute joint angle in degrees then to radians
        angle_deg = info['min_deg'] + (info['max_deg'] - info['min_deg']) * (1 - pct / 100.0)
        angle_rad = angle_deg * DEG2RAD
        self.servos['gripper'].angle = self._angle_to_servo('gripper', angle_rad)
        self.finger = pct
        self.logger.info("Gripper set to %.1f%%", pct)

    def get_position(self) -> Tuple[float, float, float]:
        """Return the current (x,y,z) of the end effector."""
        return (self.x, self.y, self.z)

    def get_finger(self) -> float:
        """Return the current gripper opening percentage."""
        return self.finger
    