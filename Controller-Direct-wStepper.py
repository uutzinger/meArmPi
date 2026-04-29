#!/usr/bin/env python3
#################################################################################################################
# meArm Direct + Stepper Controller
# Uses mutually exclusive servo and stepper modes on a shared PCA9685/Motor HAT address.
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

import json
import logging
from pathlib import Path
import time

import board
import pygame
from adafruit_pca9685 import PCA9685
from adafruit_motor import stepper as stepper_mod

import meArm

#################################################################################################################
HAT_ADDRESS  = 0x6F
MODE_SERVO   = "SERVO"
MODE_STEPPER = "STEPPER"
MODE_IDLE    = "IDLE"

SERVO_STEP         = 5.0

STEPPER_RPM        = 180.0
MIN_STEPPER_RPM    = 5.0
MAX_STEPPER_RPM    = 240.0
STEPPER_ACCEL_RPM_PER_SEC = 240.0
STEPPER_DECEL_RPM_PER_SEC = 1200.0
STEPS_PER_REV      = 200
STEPPER_STYLE      = "SINGLE"    # "SINGLE", "DOUBLE", "INTERLEAVE", or "MICROSTEP"
STYLE_STEP_FACTOR  = 1.0         # 1.0 for SINGLE/DOUBLE, 2.0 for INTERLEAVE, 16.0 for MICROSTEP (assuming 16 microsteps per full step)
MAX_STEPS_PER_TICK = 12

INTERVAL_USERINPUT = 0.03
INTERVAL_MOTOR     = 0.01
INTERVAL_BLINK     = 0.35
JOYTHRESH          = 0.01
MODE_SWITCH_COOLDOWN = 0.2
PCA_SETTLE_DELAY   = 0.02

WINDOW_SIZE        = (540, 420)
SERVO_PWM_FREQUENCY = 50
STEPPER_PWM_FREQUENCY = 1600

WAYPOINT_FILE = Path(__file__).with_name("controller_direct_waypoints.json")
WAYPOINT_BUTTONS = {
    3: "square",
    0: "x",
    1: "circle",
    2: "triangle",
}
PROGRAM_BUTTONS = {8, 6}    # share/select on common PlayStation/generic mappings
RUN_BUTTONS = {9, 7}        # options/start on common PlayStation/generic mappings
L1_BUTTONS = {4}
R1_BUTTONS = {5}

STEPPER2_ENABLE_CHANNELS = (2, 7)
STEPPER2_COIL_CHANNELS = (4, 3, 5, 6)
STEPPER_SEQUENCE_SINGLE = (
    (1, 0, 0, 0),
    (0, 0, 1, 0),
    (0, 1, 0, 0),
    (0, 0, 0, 1),
)
STEPPER_SEQUENCE_DOUBLE = (
    (1, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 1),
    (1, 0, 0, 1),
)
STEPPER_SEQUENCE_INTERLEAVE = (
    (1, 0, 0, 0),
    (1, 0, 1, 0),
    (0, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 0),
    (0, 1, 0, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
)
#################################################################################################################


class HatStepper:
    """Minimal M3/M4 stepper wrapper on a shared PCA9685."""

    def __init__(self, pca, enable_channels, coil_channels):
        self.pca = pca
        self.enable_channels = enable_channels
        self.coils = [self.pca.channels[channel] for channel in coil_channels]
        self.step_index = 0
        self._set_enabled(False)
        self.release()

    def _set_enabled(self, enabled):
        duty_cycle = 0xFFFF if enabled else 0
        for channel in self.enable_channels:
            self.pca.channels[channel].duty_cycle = duty_cycle

    def _sequence(self, style):
        if style == stepper_mod.SINGLE:
            return STEPPER_SEQUENCE_SINGLE
        if style == stepper_mod.DOUBLE:
            return STEPPER_SEQUENCE_DOUBLE
        if style == stepper_mod.INTERLEAVE:
            return STEPPER_SEQUENCE_INTERLEAVE
        raise AttributeError(f"Unsupported step style: {STEPPER_STYLE}")

    def onestep(self, *, direction=stepper_mod.FORWARD, style=stepper_mod.SINGLE):
        sequence = self._sequence(style)
        delta = 1 if direction == stepper_mod.FORWARD else -1
        self.step_index = (self.step_index + delta) % len(sequence)
        self._set_enabled(True)
        for coil, value in zip(self.coils, sequence[self.step_index]):
            coil.duty_cycle = 0xFFFF if value else 0

    def release(self):
        for coil in self.coils:
            coil.duty_cycle = 0
        self._set_enabled(False)


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


def axis_value(axes, index, default=0.0):
    """Return an axis value if the controller exposes it."""
    if index < len(axes):
        return axes[index]
    return default


def waypoint_position(base, shoulder, elbow, gripper):
    """Return a serializable waypoint position."""
    return {
        "base": base,
        "shoulder": shoulder,
        "elbow": elbow,
        "gripper": gripper,
    }


def load_waypoints(logger):
    """Load direct-controller waypoints from JSON."""
    waypoints = {name: None for name in WAYPOINT_BUTTONS.values()}
    if not WAYPOINT_FILE.exists():
        return waypoints

    try:
        with WAYPOINT_FILE.open() as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        logger.warning("Could not load %s: %s", WAYPOINT_FILE, exc)
        return waypoints

    for name in waypoints:
        position = data.get(name)
        if isinstance(position, dict):
            try:
                waypoints[name] = {
                    "base": float(position["base"]),
                    "shoulder": float(position["shoulder"]),
                    "elbow": float(position["elbow"]),
                    "gripper": float(position["gripper"]),
                }
            except (KeyError, TypeError, ValueError):
                logger.warning("Ignoring invalid waypoint %s in %s", name, WAYPOINT_FILE)

    return waypoints


def save_waypoints(waypoints, logger):
    """Save direct-controller waypoints to JSON."""
    try:
        with WAYPOINT_FILE.open("w") as f:
            json.dump(waypoints, f, indent=4)
            f.write("\n")
    except OSError as exc:
        logger.warning("Could not save %s: %s", WAYPOINT_FILE, exc)


def handle_waypoint_button(
    button,
    base,
    shoulder,
    elbow,
    gripper,
    waypoints,
    programming_mode,
    logger,
):
    """Store or recall a waypoint from a face-button press."""
    name = WAYPOINT_BUTTONS.get(button)
    status = None
    if name is None:
        return base, shoulder, elbow, gripper, status

    if programming_mode:
        waypoints[name] = waypoint_position(base, shoulder, elbow, gripper)
        save_waypoints(waypoints, logger)
        status = f"Saved {name} waypoint"
        logger.info(status)
        return base, shoulder, elbow, gripper, status

    position = waypoints.get(name)
    if position is None:
        status = f"No {name} waypoint saved"
        logger.info(status)
        return base, shoulder, elbow, gripper, status

    status = f"Loaded {name} waypoint"
    logger.info(status)
    return (
        position["base"],
        position["shoulder"],
        position["elbow"],
        position["gripper"],
        status,
    )


def check_servo_keys(base, shoulder, elbow, gripper, logger):
    """Poll keyboard for direct servo control."""
    keys = pygame.key.get_pressed()
    if keys[pygame.K_RIGHT]:
        base += SERVO_STEP
        logger.debug("Base increase")
    elif keys[pygame.K_LEFT]:
        base -= SERVO_STEP
        logger.debug("Base decrease")
    elif keys[pygame.K_UP]:
        shoulder += SERVO_STEP
        logger.debug("Shoulder increase")
    elif keys[pygame.K_DOWN]:
        shoulder -= SERVO_STEP
        logger.debug("Shoulder decrease")
    elif keys[pygame.K_w]:
        elbow += SERVO_STEP
        logger.debug("Elbow increase")
    elif keys[pygame.K_s]:
        elbow -= SERVO_STEP
        logger.debug("Elbow decrease")
    elif keys[pygame.K_d]:
        gripper += SERVO_STEP
        logger.debug("Gripper increase")
    elif keys[pygame.K_a]:
        gripper -= SERVO_STEP
        logger.debug("Gripper decrease")

    return base, shoulder, elbow, gripper


def check_servo_joy_axis(joystick, base, shoulder, elbow, gripper, logger):
    """Handle direct joint-angle input from joystick axes."""
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    joy = False

    base_axis = axis_value(axes, 0)
    shoulder_axis = axis_value(axes, 1)
    elbow_axis = axis_value(axes, 4)
    left_trigger = axis_value(axes, 2, -1.0)
    right_trigger = axis_value(axes, 5, -1.0)

    if abs(base_axis) > JOYTHRESH:
        base += base_axis * SERVO_STEP
        joy = True
    if abs(shoulder_axis) > JOYTHRESH:
        shoulder -= shoulder_axis * SERVO_STEP
        joy = True
    if abs(elbow_axis) > JOYTHRESH:
        elbow -= elbow_axis * SERVO_STEP
        joy = True
    if left_trigger > -1.0 or right_trigger > -1.0:
        gripper += ((left_trigger + 1.0) / 2.0 - (right_trigger + 1.0) / 2.0) * SERVO_STEP
        joy = True

    if joy:
        logger.debug(
            "Joystick base:%.0f shoulder:%.0f elbow:%.0f gripper:%.0f",
            base,
            shoulder,
            elbow,
            gripper,
        )

    return base, shoulder, elbow, gripper


def check_servo_joy_hat(joystick, base, shoulder, elbow, gripper, logger):
    """Handle direct gripper input from the joystick D-pad/hat."""
    if joystick.get_numhats() < 1:
        return base, shoulder, elbow, gripper

    hat_x, _ = joystick.get_hat(0)
    if hat_x > 0:
        gripper += SERVO_STEP
        logger.debug("Joystick hat gripper increase")
    elif hat_x < 0:
        gripper -= SERVO_STEP
        logger.debug("Joystick hat gripper decrease")

    return base, shoulder, elbow, gripper


def get_style_constant():
    """Resolve the configured step style constant from adafruit_motor.stepper."""
    return getattr(stepper_mod, STEPPER_STYLE, None)


def step_once(stepper_motor, direction):
    """Move one step using the CircuitPython stepper API."""
    style = get_style_constant()
    if style is None:
        raise AttributeError(f"Unsupported step style: {STEPPER_STYLE}")
    stepper_motor.onestep(direction=direction, style=style)


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
    target_rpm = clamp(command_rpm, -MAX_STEPPER_RPM, MAX_STEPPER_RPM)
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
    rpm = clamp(abs(current_rpm), MIN_STEPPER_RPM, MAX_STEPPER_RPM)
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


def stepper_keyboard_command():
    """Read keyboard commands for the M3/M4 stepper in stepper mode."""
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        return STEPPER_RPM
    if keys[pygame.K_s]:
        return -STEPPER_RPM
    return 0.0


def button_pressed(joystick, button_numbers):
    """Return True when any listed joystick button exists and is pressed."""
    button_count = joystick.get_numbuttons()
    for button in button_numbers:
        if button < button_count and joystick.get_button(button):
            return True
    return False


def is_stepper_button(button):
    """Return True when the button should switch to stepper mode."""
    return button in L1_BUTTONS or button in R1_BUTTONS


def is_servo_button(button):
    """Return True when the button should switch to servo mode."""
    servo_buttons = set(WAYPOINT_BUTTONS) | PROGRAM_BUTTONS | RUN_BUTTONS
    return button in servo_buttons


def servo_joystick_requested(joystick):
    """Detect joystick servo input so controls can switch back to servo mode."""
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    if abs(axis_value(axes, 0)) > JOYTHRESH:
        return True
    if abs(axis_value(axes, 1)) > JOYTHRESH:
        return True
    if abs(axis_value(axes, 4)) > JOYTHRESH:
        return True
    if axis_value(axes, 2, -1.0) > -1.0:
        return True
    if axis_value(axes, 5, -1.0) > -1.0:
        return True
    if joystick.get_numhats() > 0:
        hat_x, _ = joystick.get_hat(0)
        if hat_x != 0:
            return True
    return False


def stepper_joystick_command(joystick):
    """Read L1/R1 commands for the M3/M4 stepper in stepper mode."""
    command_rpm = 0.0
    if button_pressed(joystick, R1_BUTTONS):
        command_rpm += STEPPER_RPM
    if button_pressed(joystick, L1_BUTTONS):
        command_rpm -= STEPPER_RPM
    return command_rpm


def release_stepper(stepper_motor):
    """Release holding torque when a stepper object exists."""
    if stepper_motor is not None:
        stepper_motor.release()


def set_pca_frequency(pca, frequency_hz):
    """Retune the shared PCA9685 and allow the outputs to settle."""
    pca.frequency = frequency_hz
    time.sleep(PCA_SETTLE_DELAY)


def deinit_pca(pca):
    """Release the shared PCA object when the platform exposes a deinit hook."""
    deinit = getattr(pca, "deinit", None)
    if deinit is not None:
        deinit()


def deinit_i2c(i2c):
    """Release the shared I2C bus when the platform exposes a deinit hook."""
    deinit = getattr(i2c, "deinit", None)
    if deinit is not None:
        deinit()


def exit_servo_mode(state, logger):
    """Save servo state and release PWM outputs."""
    arm = state["arm"]
    if arm is None:
        return

    state["base"], state["shoulder"], state["elbow"] = arm.get_joint_angles()
    state["gripper"] = arm.get_gripper_angle()
    arm.deinit()
    state["arm"] = None
    logger.info("Exited servo mode")


def enter_servo_mode(state, logger):
    """Initialize servo control at 50 Hz and restore the last servo angles."""
    if state["mode"] == MODE_SERVO:
        return

    set_pca_frequency(state["pca"], SERVO_PWM_FREQUENCY)
    home_on_start = not state["servo_started"]
    arm = meArm.meArm(
        i2c=state["i2c"],
        pca=state["pca"],
        address=HAT_ADDRESS,
        logger=logger,
        home_on_start=home_on_start,
    )
    state["arm"] = arm
    state["servo_started"] = True

    if home_on_start:
        state["base"], state["shoulder"], state["elbow"] = arm.get_joint_angles()
        state["gripper"] = arm.get_gripper_angle()
    else:
        state["base"], state["shoulder"], state["elbow"] = arm.set_joint_angles(
            state["base"],
            state["shoulder"],
            state["elbow"],
        )
        state["gripper"] = arm.set_gripper_angle(state["gripper"])

    state["mode"] = MODE_SERVO
    state["pending_mode"] = None
    state["last_mode_switch_time"] = time.time()
    state["status"] = "Servo mode: direct arm control active"
    logger.info("Entered servo mode")


def exit_stepper_mode(state, logger):
    """Release the M3/M4 stepper."""
    release_stepper(state["stepper"])
    state["stepper"] = None
    state["stepper_state"]["command_rpm"] = 0.0
    state["stepper_state"]["target_rpm"] = 0.0
    state["stepper_state"]["current_rpm"] = 0.0
    state["stepper_state"]["energized"] = False
    state["stepper_state"]["last_ramp_time"] = time.time()
    logger.info("Exited stepper mode")


def enter_stepper_mode(state, logger):
    """Release servo outputs and initialize the shared-PCA M3/M4 stepper."""
    if state["mode"] == MODE_STEPPER:
        return

    set_pca_frequency(state["pca"], STEPPER_PWM_FREQUENCY)
    state["stepper"] = HatStepper(state["pca"], STEPPER2_ENABLE_CHANNELS, STEPPER2_COIL_CHANNELS)
    current_time = time.time()
    state["stepper_state"]["last_step_time"] = current_time
    state["stepper_state"]["last_ramp_time"] = current_time
    state["stepper_state"]["command_rpm"] = 0.0
    state["stepper_state"]["target_rpm"] = 0.0
    state["stepper_state"]["current_rpm"] = 0.0
    state["stepper_state"]["energized"] = False
    state["mode"] = MODE_STEPPER
    state["pending_mode"] = None
    state["last_mode_switch_time"] = current_time
    state["programming_mode"] = False
    state["status"] = "Stepper mode: hold L1/R1 for M3/M4 stepper"
    logger.info("Entered stepper mode")


def enter_idle_mode(state, logger):
    """Release all hardware outputs."""
    if state["mode"] == MODE_SERVO:
        exit_servo_mode(state, logger)
    elif state["mode"] == MODE_STEPPER:
        exit_stepper_mode(state, logger)

    state["mode"] = MODE_IDLE
    state["pending_mode"] = None
    state["last_mode_switch_time"] = time.time()
    state["programming_mode"] = False
    state["status"] = "Idle mode: outputs released"
    logger.info("Entered idle mode")


def shutdown_hardware(state, logger):
    """Release whichever hardware mode currently owns the HAT."""
    if state["mode"] == MODE_SERVO:
        exit_servo_mode(state, logger)
    elif state["mode"] == MODE_STEPPER:
        exit_stepper_mode(state, logger)
    state["mode"] = MODE_IDLE
    deinit_pca(state["pca"])
    deinit_i2c(state["i2c"])


def update_text(state, screen, font, font_small):
    """Render the combined controller state."""
    mode = state["mode"]
    programming_mode = state["programming_mode"]
    blink_on = int(time.time() / INTERVAL_BLINK) % 2 == 0

    screen.fill((255, 255, 255))
    title = font.render(f"Mode: {mode}", True, (0, 0, 0))
    status = font_small.render(state["status"], True, (0, 0, 0))
    screen.blit(title, (10, 10))
    screen.blit(status, (10, 42))

    if mode == MODE_SERVO:
        mode_color = (0, 120, 0)
    elif mode == MODE_STEPPER:
        mode_color = (0, 70, 200)
    else:
        mode_color = (160, 160, 160)
    pygame.draw.circle(screen, mode_color, (500, 24), 10)
    pygame.draw.circle(screen, (0, 0, 0), (500, 24), 10, 2)

    if programming_mode:
        led_color = (220, 0, 0) if blink_on else (255, 255, 255)
        led_outline = (220, 0, 0)
    else:
        led_color = (180, 180, 180)
        led_outline = (120, 120, 120)
    pygame.draw.circle(screen, led_color, (500, 50), 8)
    pygame.draw.circle(screen, led_outline, (500, 50), 8, 2)

    servo_1 = font_small.render(
        f"Servo base/shoulder/elbow: {state['base']:.0f} / {state['shoulder']:.0f} / {state['elbow']:.0f}",
        True,
        (0, 0, 0),
    )
    servo_2 = font_small.render(f"Servo gripper: {state['gripper']:.0f}", True, (0, 0, 0))
    stepper_1 = font_small.render(
        f"M3/M4 stepper: {state['stepper_state']['position_steps']:7d} steps  "
        f"{state['stepper_state']['current_rpm']:6.1f}/{state['stepper_state']['target_rpm']:6.1f} rpm",
        True,
        (0, 0, 0),
    )
    screen.blit(servo_1, (10, 80))
    screen.blit(servo_2, (10, 102))
    screen.blit(stepper_1, (10, 132))

    help_lines = [
        "Modes: Auto or 1=servo  2=stepper  0=idle/release",
        "Servo keys:     arrows base/shoulder, W/S elbow, A/D gripper",
        "Servo joystick: axis 0/1/4 joints, triggers or D-pad L/R gripper",
        "Stepper:        hold R1 forward, L1 backward on M3/M4; rpm shown actual/target",
        f"PWM freq:       servo {SERVO_PWM_FREQUENCY} Hz, stepper {STEPPER_PWM_FREQUENCY} Hz",
        "Mode switch:    200 ms lockout; stepper must ramp down before servo/idle",
        "Waypoints: Square/X/Circle/Triangle; left center program, right center run",
    ]
    for index, line in enumerate(help_lines):
        rendered = font_small.render(line, True, (0, 0, 0))
        screen.blit(rendered, (10, 190 + index * 24))

    pygame.display.flip()


def create_state(logger):
    """Build mutable combined-controller state."""
    current_time = time.time()
    i2c = board.I2C()
    pca = PCA9685(i2c, address=HAT_ADDRESS)
    pca.frequency = SERVO_PWM_FREQUENCY
    return {
        "i2c": i2c,
        "pca": pca,
        "mode": MODE_IDLE,
        "arm": None,
        "stepper": None,
        "servo_started": False,
        "pending_mode": None,
        "pending_servo_button": None,
        "last_mode_switch_time": 0.0,
        "base": 0.0,
        "shoulder": 0.0,
        "elbow": 0.0,
        "gripper": 0.0,
        "waypoints": load_waypoints(logger),
        "programming_mode": False,
        "status": "Starting",
        "stepper_state": {
            "position_steps": 0,
            "command_rpm": 0.0,
            "target_rpm": 0.0,
            "current_rpm": 0.0,
            "energized": False,
            "last_step_time": current_time,
            "last_ramp_time": current_time,
        },
    }


def request_mode(target_mode, state, logger):
    """Queue a mode transition, respecting cooldown and stepper ramp-down."""
    if target_mode == state["mode"] and state["pending_mode"] is None:
        return False

    if target_mode == state["pending_mode"]:
        return False

    state["pending_mode"] = target_mode
    if state["mode"] == MODE_STEPPER and target_mode != MODE_STEPPER:
        state["status"] = f"Stepper mode: decelerating to enter {target_mode.lower()}"
    else:
        state["status"] = f"Requested {target_mode.lower()} mode"
    logger.info("Requested mode change to %s", target_mode)
    return True


def perform_mode_switch(target_mode, state, logger):
    """Switch modes immediately after teardown of the current owner."""
    if state["mode"] == MODE_SERVO:
        exit_servo_mode(state, logger)
    elif state["mode"] == MODE_STEPPER:
        exit_stepper_mode(state, logger)

    if target_mode == MODE_SERVO:
        enter_servo_mode(state, logger)
    elif target_mode == MODE_STEPPER:
        enter_stepper_mode(state, logger)
    else:
        enter_idle_mode(state, logger)


def process_pending_mode(state, current_time, logger):
    """Complete queued mode changes once cooldown and ramp-down constraints allow it."""
    target_mode = state["pending_mode"]
    if target_mode is None:
        return False

    if (current_time - state["last_mode_switch_time"]) < MODE_SWITCH_COOLDOWN:
        return False

    if state["mode"] == MODE_STEPPER and target_mode != MODE_STEPPER:
        if state["stepper_state"]["current_rpm"] != 0.0:
            state["status"] = f"Stepper mode: decelerating to enter {target_mode.lower()}"
            return False

    perform_mode_switch(target_mode, state, logger)
    return True


def handle_mode_key(event, state, logger):
    """Handle keyboard mode switches."""
    if event.key == pygame.K_1:
        return request_mode(MODE_SERVO, state, logger)
    if event.key == pygame.K_2:
        return request_mode(MODE_STEPPER, state, logger)
    if event.key == pygame.K_0:
        return request_mode(MODE_IDLE, state, logger)
    return False


def handle_servo_button(button, state, logger):
    """Handle servo-mode gamepad button functions."""
    if button in PROGRAM_BUTTONS:
        state["programming_mode"] = True
        state["status"] = "Programming: press waypoint button to save"
        return True
    if button in RUN_BUTTONS:
        state["programming_mode"] = False
        state["status"] = "Run mode: press waypoint button to move"
        return True

    (
        state["base"],
        state["shoulder"],
        state["elbow"],
        state["gripper"],
        waypoint_status,
    ) = handle_waypoint_button(
        button,
        state["base"],
        state["shoulder"],
        state["elbow"],
        state["gripper"],
        state["waypoints"],
        state["programming_mode"],
        logger,
    )
    if waypoint_status is not None:
        state["status"] = waypoint_status
        return True
    return False


def handle_automatic_joystick_mode(joystick, joy, state, logger):
    """Switch modes from joystick controls without requiring keyboard mode keys."""
    if not joy:
        return False

    if stepper_joystick_command(joystick) != 0.0:
        if state["mode"] != MODE_STEPPER:
            return request_mode(MODE_STEPPER, state, logger)
        return False

    if state["mode"] != MODE_SERVO and servo_joystick_requested(joystick):
        return request_mode(MODE_SERVO, state, logger)

    return False


def service_servo_mode(state, joystick, joy, current_time, check_userinput_time, motor_time, logger):
    """Poll and service direct servo controls."""
    changed = False
    if (current_time - check_userinput_time) > INTERVAL_USERINPUT:
        state["base"], state["shoulder"], state["elbow"], state["gripper"] = check_servo_keys(
            state["base"],
            state["shoulder"],
            state["elbow"],
            state["gripper"],
            logger,
        )
        if joy:
            state["base"], state["shoulder"], state["elbow"], state["gripper"] = check_servo_joy_axis(
                joystick,
                state["base"],
                state["shoulder"],
                state["elbow"],
                state["gripper"],
                logger,
            )
            state["base"], state["shoulder"], state["elbow"], state["gripper"] = check_servo_joy_hat(
                joystick,
                state["base"],
                state["shoulder"],
                state["elbow"],
                state["gripper"],
                logger,
            )
        check_userinput_time = current_time

    arm = state["arm"]
    if arm is not None and (current_time - motor_time) > INTERVAL_MOTOR:
        if (state["base"], state["shoulder"], state["elbow"]) != arm.get_joint_angles():
            state["base"], state["shoulder"], state["elbow"] = arm.set_joint_angles(
                state["base"],
                state["shoulder"],
                state["elbow"],
            )
            changed = True

        if state["gripper"] != arm.get_gripper_angle():
            state["gripper"] = arm.set_gripper_angle(state["gripper"])
            changed = True

        motor_time = current_time

    return check_userinput_time, motor_time, changed


def service_stepper_mode(state, joystick, joy, current_time):
    """Poll and service the M3/M4 stepper."""
    command_rpm = stepper_keyboard_command()
    if joy and command_rpm == 0.0:
        command_rpm = stepper_joystick_command(joystick)

    if state["pending_mode"] in {MODE_SERVO, MODE_IDLE}:
        command_rpm = 0.0

    previous_rpm = state["stepper_state"]["command_rpm"]
    changed = False
    if state["stepper"] is not None:
        changed = service_stepper(state["stepper"], command_rpm, state["stepper_state"], current_time)

    return changed or previous_rpm != state["stepper_state"]["command_rpm"]


def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("meArm Direct + Stepper Controller")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 32)
    font_small = pygame.font.SysFont(None, 20)

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        logger.info("Controller detected: %s", joystick.get_name())
        joy = True
    except pygame.error:
        logger.warning("No controller detected.")
        joystick = None
        joy = False

    state = create_state(logger)
    enter_servo_mode(state, logger)

    current_time = time.time()
    motor_time = current_time
    check_userinput_time = current_time
    blink_time = current_time
    update_text(state, screen, font, font_small)

    running = True
    try:
        while running:
            current_time = time.time()
            redraw = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if handle_mode_key(event, state, logger):
                        redraw = True
                elif event.type == pygame.JOYBUTTONDOWN:
                    if is_stepper_button(event.button):
                        redraw = request_mode(MODE_STEPPER, state, logger) or redraw
                    elif is_servo_button(event.button):
                        state["pending_servo_button"] = event.button
                        if state["mode"] != MODE_SERVO:
                            redraw = request_mode(MODE_SERVO, state, logger) or redraw
                        else:
                            redraw = True

            if process_pending_mode(state, current_time, logger):
                redraw = True

            if state["mode"] == MODE_SERVO and state["pending_servo_button"] is not None:
                if handle_servo_button(state["pending_servo_button"], state, logger):
                    redraw = True
                state["pending_servo_button"] = None

            if handle_automatic_joystick_mode(joystick, joy, state, logger):
                redraw = True

            if state["mode"] == MODE_SERVO:
                check_userinput_time, motor_time, changed = service_servo_mode(
                    state,
                    joystick,
                    joy,
                    current_time,
                    check_userinput_time,
                    motor_time,
                    logger,
                )
                redraw = redraw or changed
            elif state["mode"] == MODE_STEPPER:
                redraw = redraw or service_stepper_mode(state, joystick, joy, current_time)

            if state["programming_mode"] and (current_time - blink_time) > INTERVAL_BLINK:
                redraw = True
                blink_time = current_time

            if redraw:
                update_text(state, screen, font, font_small)

            clock.tick(120)
    finally:
        shutdown_hardware(state, logger)
        pygame.quit()


if __name__ == "__main__":
    main()
