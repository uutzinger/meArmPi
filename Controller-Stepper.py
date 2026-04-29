#!/usr/bin/env python3
#################################################################################################################
# meArm Stepper Controller
# Uses keyboard or gamepad to control two stepper motors on an Adafruit Motor HAT
#################################################################################################################

# ##############################################################################
#
# The MIT License (MIT)
#
# Copyright (c) 2026 Urs Utzinger
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

import logging
import time

import board
import pygame
from adafruit_motor import stepper as stepper_mod
from adafruit_motorkit import MotorKit

#################################################################################################################
HAT_ADDRESS        = 0x6F
I2C_BUS            = 1
STEPS_PER_REV      = 200
KEYBOARD_RPM       = 30.0
MIN_JOYSTICK_RPM   = 5.0
MAX_JOYSTICK_RPM   = 90.0
STEPPER_ACCEL_RPM_PER_SEC = 240.0
STEPPER_DECEL_RPM_PER_SEC = 1200.0
JOYTHRESH          = 0.10
INTERVAL_USERINPUT = 0.03
WINDOW_SIZE        = (520, 360)
STEPPER_STYLE      = "INTERLEAVE"
STYLE_STEP_FACTOR  = 2.0
MAX_STEPS_PER_TICK = 8
LEFT_Y_AXIS        = 1
RIGHT_Y_AXIS       = 4
#################################################################################################################


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


def axis_value(axes, index, default=0.0):
    """Return an axis value if the controller exposes it."""
    if index < len(axes):
        return axes[index]
    return default


def axis_to_rpm(axis):
    """Map joystick displacement to signed RPM with a deadband around zero."""
    magnitude = abs(axis)
    if magnitude <= JOYTHRESH:
        return 0.0
    scaled = (magnitude - JOYTHRESH) / (1.0 - JOYTHRESH)
    rpm = MIN_JOYSTICK_RPM + scaled * (MAX_JOYSTICK_RPM - MIN_JOYSTICK_RPM)
    return rpm if axis > 0 else -rpm


def keyboard_commands():
    """Read keyboard commands for the two steppers."""
    keys = pygame.key.get_pressed()
    motor_1_rpm = 0.0
    motor_2_rpm = 0.0

    if keys[pygame.K_UP]:
        motor_1_rpm = KEYBOARD_RPM
    elif keys[pygame.K_DOWN]:
        motor_1_rpm = -KEYBOARD_RPM

    if keys[pygame.K_w]:
        motor_2_rpm = KEYBOARD_RPM
    elif keys[pygame.K_s]:
        motor_2_rpm = -KEYBOARD_RPM

    return motor_1_rpm, motor_2_rpm


def joystick_commands(joystick):
    """Read joystick commands for the two steppers."""
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    left_y = -axis_value(axes, LEFT_Y_AXIS)
    right_y = -axis_value(axes, RIGHT_Y_AXIS)
    return axis_to_rpm(left_y), axis_to_rpm(right_y), left_y, right_y


def combine_commands(keyboard_rpm, joystick_rpm):
    """Let keyboard override joystick when a keyboard command is active."""
    if keyboard_rpm != 0.0:
        return keyboard_rpm
    return joystick_rpm


def get_style_constant():
    """Resolve the configured step style constant from adafruit_motor.stepper."""
    return getattr(stepper_mod, STEPPER_STYLE, None)


def step_once(stepper_motor, direction):
    """Move one step using the CircuitPython stepper API."""
    style = get_style_constant()
    if style is None:
        raise AttributeError(f"Unsupported step style: {STEPPER_STYLE}")
    stepper_motor.onestep(direction=direction, style=style)


def release_stepper(stepper_motor):
    """Release holding torque."""
    stepper_motor.release()


def rpm_to_interval(rpm):
    """Convert RPM to the time interval between service steps."""
    return 60.0 / (STEPS_PER_REV * STYLE_STEP_FACTOR * rpm)


def ramp_rpm(current_rpm, target_rpm, elapsed):
    """Move the current RPM toward the target RPM using acceleration or braking."""
    braking = target_rpm == 0.0 or current_rpm * target_rpm < 0.0 or abs(target_rpm) < abs(current_rpm)
    rate = STEPPER_DECEL_RPM_PER_SEC if braking else STEPPER_ACCEL_RPM_PER_SEC
    max_delta = rate * elapsed
    delta = target_rpm - current_rpm
    if abs(delta) <= max_delta:
        return target_rpm
    if delta > 0.0:
        return current_rpm + max_delta
    return current_rpm - max_delta


def service_stepper(stepper_motor, command_rpm, state, current_time):
    """Advance a stepper based on the commanded RPM."""
    target_rpm = clamp(command_rpm, -MAX_JOYSTICK_RPM, MAX_JOYSTICK_RPM)
    state["target_rpm"] = target_rpm

    ramp_elapsed = current_time - state["last_ramp_time"]
    previous_rpm = state["current_rpm"]
    current_rpm = ramp_rpm(previous_rpm, target_rpm, ramp_elapsed)
    state["current_rpm"] = current_rpm
    state["command_rpm"] = current_rpm
    state["last_ramp_time"] = current_time
    speed_changed = current_rpm != previous_rpm

    if current_rpm == 0.0:
        if state["energized"]:
            release_stepper(stepper_motor)
            state["energized"] = False
        state["last_step_time"] = current_time
        return speed_changed

    direction = stepper_mod.FORWARD if current_rpm > 0.0 else stepper_mod.BACKWARD
    rpm = clamp(abs(current_rpm), MIN_JOYSTICK_RPM, MAX_JOYSTICK_RPM)
    interval = rpm_to_interval(rpm)
    elapsed = current_time - state["last_step_time"]

    if elapsed < interval:
        return speed_changed

    steps_due = clamp(int(elapsed / interval), 1, MAX_STEPS_PER_TICK)
    delta = 1 if direction == stepper_mod.FORWARD else -1

    for _ in range(steps_due):
        step_once(stepper_motor, direction)
        state["position_steps"] += delta
        state["last_step_time"] += interval

    state["energized"] = True

    if current_time - state["last_step_time"] > interval:
        state["last_step_time"] = current_time

    return True


def update_text(stepper_1_state, stepper_2_state, joystick_state, screen, font, font_small):
    """Render motor positions and commanded speeds."""
    text0 = font.render(
        f"Stepper 1: {stepper_1_state['position_steps']:7d} steps  "
        f"{stepper_1_state['current_rpm']:6.1f}/{stepper_1_state['target_rpm']:6.1f} rpm",
        True,
        (0, 0, 0),
    )
    text1 = font.render(
        f"Stepper 2: {stepper_2_state['position_steps']:7d} steps  "
        f"{stepper_2_state['current_rpm']:6.1f}/{stepper_2_state['target_rpm']:6.1f} rpm",
        True,
        (0, 0, 0),
    )

    help_1 = font_small.render("Keyboard Up/Down: Stepper 1 +/-", True, (0, 0, 0))
    help_2 = font_small.render("Keyboard W/S: Stepper 2 +/-", True, (0, 0, 0))
    help_3 = font_small.render("Left joystick up/down: Stepper 1 variable speed", True, (0, 0, 0))
    help_4 = font_small.render("Right joystick up/down: Stepper 2 variable speed", True, (0, 0, 0))
    help_5 = font_small.render(
        f"Style: {STEPPER_STYLE}  Steps/rev: {STEPS_PER_REV}  Hat: 0x{HAT_ADDRESS:02X}",
        True,
        (0, 0, 0),
    )
    help_6 = font_small.render(
        f"Joystick speed: {MIN_JOYSTICK_RPM:.0f}-{MAX_JOYSTICK_RPM:.0f} rpm  accel/decel: "
        f"{STEPPER_ACCEL_RPM_PER_SEC:.0f}/{STEPPER_DECEL_RPM_PER_SEC:.0f}",
        True,
        (0, 0, 0),
    )
    help_7 = font_small.render(
        f"Raw pygame axis: left[{LEFT_Y_AXIS}]={joystick_state['left_y']:+.2f}  right[{RIGHT_Y_AXIS}]={joystick_state['right_y']:+.2f}",
        True,
        (0, 0, 0),
    )
    help_8 = font_small.render("Typical pygame stick range is about -1.00 to +1.00", True, (0, 0, 0))

    screen.fill((255, 255, 255))
    screen.blit(text0, (10, 20))
    screen.blit(text1, (10, 60))
    screen.blit(help_1, (10, 130))
    screen.blit(help_2, (10, 150))
    screen.blit(help_3, (10, 190))
    screen.blit(help_4, (10, 210))
    screen.blit(help_5, (10, 250))
    screen.blit(help_6, (10, 270))
    screen.blit(help_7, (10, 290))
    screen.blit(help_8, (10, 310))
    pygame.display.flip()


def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    i2c = board.I2C()
    kit = MotorKit(i2c=i2c, address=HAT_ADDRESS)
    stepper_1 = kit.stepper1
    stepper_2 = kit.stepper2

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("meArm Stepper Controller")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 30)
    font_small = pygame.font.SysFont(None, 20)

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        logger.info("Controller detected: %s", joystick.get_name())
        joy = True
    except pygame.error:
        logger.warning("No controller detected.")
        joy = False

    current_time = time.time()
    stepper_1_state = {
        "position_steps": 0,
        "command_rpm": 0.0,
        "target_rpm": 0.0,
        "current_rpm": 0.0,
        "energized": False,
        "last_step_time": current_time,
        "last_ramp_time": current_time,
    }
    stepper_2_state = {
        "position_steps": 0,
        "command_rpm": 0.0,
        "target_rpm": 0.0,
        "current_rpm": 0.0,
        "energized": False,
        "last_step_time": current_time,
        "last_ramp_time": current_time,
    }
    joystick_state = {"motor_1_rpm": 0.0, "motor_2_rpm": 0.0, "left_y": 0.0, "right_y": 0.0}

    update_text(stepper_1_state, stepper_2_state, joystick_state, screen, font, font_small)

    check_userinput_time = current_time
    running = True
    try:
        while running:
            current_time = time.time()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            keyboard_1_rpm, keyboard_2_rpm = keyboard_commands()
            joystick_1_rpm = joystick_state["motor_1_rpm"]
            joystick_2_rpm = joystick_state["motor_2_rpm"]
            previous_left_y = joystick_state["left_y"]
            previous_right_y = joystick_state["right_y"]

            if joy and (current_time - check_userinput_time) > INTERVAL_USERINPUT:
                joystick_1_rpm, joystick_2_rpm, left_y, right_y = joystick_commands(joystick)
                joystick_state["motor_1_rpm"] = joystick_1_rpm
                joystick_state["motor_2_rpm"] = joystick_2_rpm
                joystick_state["left_y"] = left_y
                joystick_state["right_y"] = right_y
                check_userinput_time = current_time

            command_1_rpm = combine_commands(keyboard_1_rpm, joystick_1_rpm)
            command_2_rpm = combine_commands(keyboard_2_rpm, joystick_2_rpm)
            previous_1_rpm = stepper_1_state["command_rpm"]
            previous_2_rpm = stepper_2_state["command_rpm"]

            changed_1 = service_stepper(stepper_1, command_1_rpm, stepper_1_state, current_time)
            changed_2 = service_stepper(stepper_2, command_2_rpm, stepper_2_state, current_time)

            if (
                changed_1
                or changed_2
                or previous_1_rpm != stepper_1_state["command_rpm"]
                or previous_2_rpm != stepper_2_state["command_rpm"]
                or previous_left_y != joystick_state["left_y"]
                or previous_right_y != joystick_state["right_y"]
            ):
                update_text(stepper_1_state, stepper_2_state, joystick_state, screen, font, font_small)

            clock.tick(120)
    finally:
        release_stepper(stepper_1)
        release_stepper(stepper_2)
        pygame.quit()


if __name__ == "__main__":
    main()
