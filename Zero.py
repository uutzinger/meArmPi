####################################################################################
# Zero.py: Calibration utility for generating meArm JSON config.
####################################################################################
#
# Interactive tool to adjust and save servo zero offsets and per-joint limits in
# `mearm_config.json`, compatible with `MeArm`'s expected config schema.
#
# Usage:
# - Arrow keys: adjust base and shoulder
# - a/s: adjust elbow
# - d/e: adjust gripper
# - Close window to save config
#
# Urs Utzinger, Summer 2025 
# ChatGPT, Summer 2025
####################################################################################

# ##############################################################################
# 
# The MIT License (MIT)
# 
# Copyright (c) 2025 Urs Utzinger
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
from typing import Dict

import board
import pygame
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Configuration
CONFIG_FILE = "mearm_config.json"
INCREMENT = 0.5         # degrees per key press
INTERVAL = 0.2          # seconds between adjustments
DEFAULT_ZERO = 90.0     # default zero position (degrees)

# Per-joint angular limits (degrees)
JOINT_LIMITS: Dict[str, Dict[str, float]] = {
    'base':     {'min_deg':  -90.0,  'max_deg':  90.0},
    'shoulder': {'min_deg':    0.0,  'max_deg': 150.0},
    'elbow':    {'min_deg': -180.0,  'max_deg':  30.0},
    'gripper':  {'min_deg':    0.0,  'max_deg': 125.0},
}

# Initialize I2C and PCA9685
i2c = board.I2C()
pca = PCA9685(i2c, address = 0x6F, reference_clock_speed = 25000000)
pca.frequency = 50

# Create servos mapping joint names to channels
SERVOS: Dict[str, servo.Servo] = {
    'base':     servo.Servo(pca.channels[0],  min_pulse=500, max_pulse=2500),
    'shoulder': servo.Servo(pca.channels[1],  min_pulse=500, max_pulse=2500),
    'elbow':    servo.Servo(pca.channels[14], min_pulse=500, max_pulse=2500),
    'gripper':  servo.Servo(pca.channels[15], min_pulse=500, max_pulse=2500),
}

# Track zero offsets per joint
zeros: Dict[str, float] = {joint: DEFAULT_ZERO for joint in SERVOS}
# Track previous adjustment times for rate limiting
last_times: Dict[str, float] = {joint: 0.0 for joint in SERVOS}

# Pygame setup
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("meArm Calibration")
clock = pygame.time.Clock()
font_large = pygame.font.SysFont(None, 36)
font_small = pygame.font.SysFont(None, 16)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a float between min_val and max_val."""
    return max(min_val, min(max_val, value))


def load_config() -> None:
    """Load zeros and existing limits from JSON if available, else use defaults."""
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            cfg = json.load(f)
        for joint in SERVOS:
            joint_cfg = cfg.get(joint, {})
            zeros[joint] = joint_cfg.get('zero', DEFAULT_ZERO)
            # Override per-joint min/max if saved
            if 'min_deg' in joint_cfg and 'max_deg' in joint_cfg:
                JOINT_LIMITS[joint]['min_deg'] = joint_cfg['min_deg']
                JOINT_LIMITS[joint]['max_deg'] = joint_cfg['max_deg']
    # Apply loaded zeros to servos
    for joint, srv in SERVOS.items():
        srv.angle = clamp(zeros[joint],
                          0.,
                          180.)


def save_config() -> None:
    """Save zeros and per-joint limits to JSON config for MeArm."""
    cfg = {}
    for joint in SERVOS:
        cfg[joint] = {
            'zero':    clamp(zeros[joint],
                             0.,
                             180.),
            'min_deg': JOINT_LIMITS[joint]['min_deg'],
            'max_deg': JOINT_LIMITS[joint]['max_deg'],
        }
    with open(CONFIG_FILE, 'w') as f:
        json.dump(cfg, f, indent=4)
    print(f"Saved calibration to {CONFIG_FILE}")


def update_display() -> None:
    """Render current zeros and controls on screen."""
    screen.fill((0, 0, 0))
    ypos = 20
    for joint in SERVOS:
        text = font_large.render(
            f"{joint.capitalize():<8}: {zeros[joint]:>6.1f}°  "
            f"[{JOINT_LIMITS[joint]['min_deg']:.0f}, {JOINT_LIMITS[joint]['max_deg']:.0f}]", 
            True, (255, 255, 255)
        )
        screen.blit(text, (20, ypos))
        ypos += 40
    helpers = [
        "Base: Up/Down Needs to point-> FORWARD",
        "Shoulder: Left/Right Needs to point UPWARD",
        "Elbow: a/s Needs to point HORIZONTAL",
        "Gripper: d/e Needs to be CLOSED",
        "Close window to save",
    ]
    ypos = 180
    for h in helpers:
        txt = font_small.render(h, True, (200, 200, 200))
        screen.blit(txt, (20, ypos))
        ypos += 20
    pygame.display.flip()

def adjust_joint(joint: str, delta: float) -> None:
    """Adjust the zero offset of a joint by delta degrees with rate limiting."""
    new_zero = clamp(zeros[joint] + delta,
                        0.,
                        180.)
    zeros[joint] = new_zero
    SERVOS[joint].angle = new_zero


###################################################
# Main Loop
###################################################

load_config()
last_time = time.time()

running = True
while running:
    now = time.time()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            save_config()
            running = False

    if now - last_time >= INTERVAL:

        keys = pygame.key.get_pressed()
        if  keys[pygame.K_UP]: ## Moves meArm FORWARD when "up arrow" key is pressed
            adjust_joint('base', INCREMENT)
        if keys[pygame.K_UP]:
            adjust_joint('base', INCREMENT)
        elif keys[pygame.K_DOWN]:
            adjust_joint('base', -INCREMENT)
        elif keys[pygame.K_RIGHT]:
            adjust_joint('shoulder', INCREMENT)
        elif keys[pygame.K_LEFT]:
            adjust_joint('shoulder', -INCREMENT)
        elif keys[pygame.K_a]:
            adjust_joint('elbow', INCREMENT)
        elif keys[pygame.K_s]:
            adjust_joint('elbow', -INCREMENT)
        elif keys[pygame.K_d]:
            adjust_joint('gripper', INCREMENT)
        elif keys[pygame.K_e]:
            adjust_joint('gripper', -INCREMENT)
    update_display()
    clock.tick(30)
