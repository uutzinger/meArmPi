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
from typing import Tuple

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
SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 180.0
SERVO_CHANNELS = {
    'base': 0,
    'shoulder': 1,
    'elbow': 14,
    'gripper': 15,
}

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
        home_on_start: bool = True,
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
            'base': self._setup_servo(SERVO_CHANNELS['base'], self.calib['base']),
            'shoulder': self._setup_servo(SERVO_CHANNELS['shoulder'], self.calib['shoulder']),
            'elbow': self._setup_servo(SERVO_CHANNELS['elbow'], self.calib['elbow']),
            'gripper': self._setup_servo(SERVO_CHANNELS['gripper'], self.calib['gripper']),
        }

        # Track current state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.base = 0.0
        self.shoulder = 0.0
        self.elbow = 0.0
        self.gripper = 0.0
        self.finger = 0.0

        # Home wrist and gripper when requested.
        # Combined servo/stepper controllers disable this during mode switches
        # so reinitializing the servo driver does not unexpectedly move the arm.
        if home_on_start:
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

    def _gripper_angle_from_percent(self, percent: float) -> float:
        """Convert gripper open percentage to a configured joint angle in degrees."""
        pct = clamp(percent, 0.0, 100.0)
        info = self.calib['gripper']
        return info['min_deg'] + (info['max_deg'] - info['min_deg']) * (1 - pct / 100.0)

    def _gripper_percent_from_angle(self, angle_deg: float) -> float:
        """Convert a configured gripper joint angle in degrees to open percentage."""
        info = self.calib['gripper']
        span = info['max_deg'] - info['min_deg']
        if span == 0:
            return 0.0
        return clamp((info['max_deg'] - angle_deg) * 100.0 / span, 0.0, 100.0)

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
        return clamp(cmd, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)

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

    def _update_cartesian_state(self, base_rad: float, shoulder_rad: float, elbow_rad: float) -> None:
        """Sync cached Cartesian pose from the current joint angles."""
        self.x, self.y, self.z = kinematics.forward_kinematics(base_rad, shoulder_rad, elbow_rad)

    def _update_joint_state(self, base_rad: float, shoulder_rad: float, elbow_rad: float) -> None:
        """Sync cached joint-angle state from radians."""
        self.base = base_rad * RAD2DEG
        self.shoulder = shoulder_rad * RAD2DEG
        self.elbow = elbow_rad * RAD2DEG
        self._update_cartesian_state(base_rad, shoulder_rad, elbow_rad)

    def _update_gripper_state(self, gripper_rad: float) -> None:
        """Sync cached gripper angle and percentage state from radians."""
        self.gripper = gripper_rad * RAD2DEG
        self.finger = self._gripper_percent_from_angle(self.gripper)

    def _apply_joint_angles(self, base_rad: float, shoulder_rad: float, elbow_rad: float) -> Tuple[float, float, float]:
        """Clamp, command, and store the actual joint angles reached by the servos."""
        base_rad = self._angle_limits('base', base_rad)
        shoulder_rad = self._angle_limits('shoulder', shoulder_rad)
        elbow_rad = self._angle_limits('elbow', elbow_rad)

        motor_base = self._angle_to_servo('base', base_rad)
        motor_shoulder = self._angle_to_servo('shoulder', shoulder_rad)
        motor_elbow = self._angle_to_servo('elbow', elbow_rad)

        self.servos['base'].angle = motor_base
        self.servos['shoulder'].angle = motor_shoulder
        self.servos['elbow'].angle = motor_elbow

        base_rad = self._servo_to_angle('base', motor_base)
        shoulder_rad = self._servo_to_angle('shoulder', motor_shoulder)
        elbow_rad = self._servo_to_angle('elbow', motor_elbow)
        self._update_joint_state(base_rad, shoulder_rad, elbow_rad)
        return base_rad, shoulder_rad, elbow_rad

    def _apply_gripper_angle(self, gripper_rad: float) -> float:
        """Clamp, command, and store the actual gripper angle reached by the servo."""
        gripper_rad = self._angle_limits('gripper', gripper_rad)
        motor_gripper = self._angle_to_servo('gripper', gripper_rad)
        self.servos['gripper'].angle = motor_gripper
        gripper_rad = self._servo_to_angle('gripper', motor_gripper)
        self._update_gripper_state(gripper_rad)
        return gripper_rad

    def set_joint_angles(self, base: float, shoulder: float, elbow: float) -> Tuple[float, float, float]:
        """
        Set base, shoulder, and elbow device angles in degrees.
        Returns the actual joint angles after clamping and servo-limit enforcement.
        """
        base_rad, shoulder_rad, elbow_rad = self._apply_joint_angles(
            base * DEG2RAD,
            shoulder * DEG2RAD,
            elbow * DEG2RAD,
        )
        self.logger.info(
            "Joint angles set to base=%.1f shoulder=%.1f elbow=%.1f",
            base_rad * RAD2DEG,
            shoulder_rad * RAD2DEG,
            elbow_rad * RAD2DEG,
        )
        return self.get_joint_angles()

    def set_gripper_angle(self, gripper: float) -> float:
        """
        Set gripper device angle in degrees.
        Returns the actual gripper angle after clamping and servo-limit enforcement.
        """
        gripper_rad = self._apply_gripper_angle(gripper * DEG2RAD)
        self.logger.info("Gripper angle set to %.1f", gripper_rad * RAD2DEG)
        return self.get_gripper_angle()

    def get_joint_angles(self) -> Tuple[float, float, float]:
        """Return the current base, shoulder, and elbow device angles in degrees."""
        return (self.base, self.shoulder, self.elbow)

    def get_gripper_angle(self) -> float:
        """Return the current gripper device angle in degrees."""
        return self.gripper

    def release_servos(self) -> None:
        """Disable PWM on all servo channels so another mode can own the PCA9685."""
        for channel in SERVO_CHANNELS.values():
            self.pca.channels[channel].duty_cycle = 0
        self.logger.info("Servo PWM outputs released")

    def deinit(self) -> None:
        """Release servo outputs and deinitialize the PCA9685 object when supported."""
        self.release_servos()
        deinit = getattr(self.pca, "deinit", None)
        if deinit is not None:
            deinit()

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
        self._apply_joint_angles(*sol)
        self.logger.info("Moved to (%.1f, %.1f, %.1f)", self.x, self.y, self.z)
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
        self._apply_gripper_angle(self.calib['gripper']['min_deg'] * DEG2RAD)
        self.logger.info("Gripper opened")

    def close_gripper(self) -> None:
        """Fully close the gripper."""
        self._apply_gripper_angle(self.calib['gripper']['max_deg'] * DEG2RAD)
        self.logger.info("Gripper closed")

    def partial_grip(self, percent: float) -> None:
        """
        Set gripper opening to a percentage (0=closed, 100=open).
        """
        angle_deg = self._gripper_angle_from_percent(percent)
        self._apply_gripper_angle(angle_deg * DEG2RAD)
        self.logger.info("Gripper set to %.1f%%", self.finger)

    def get_position(self) -> Tuple[float, float, float]:
        """Return the current (x,y,z) of the end effector."""
        return (self.x, self.y, self.z)

    def get_finger(self) -> float:
        """Return the current gripper opening percentage."""
        return self.finger
    
