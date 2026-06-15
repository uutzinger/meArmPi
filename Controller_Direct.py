#!/usr/bin/env python3
#################################################################################################################
# meArm Direct Controller
# Uses keyboard or gamepad to control meArm joint angles directly without inverse kinematics.
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

import pygame

import meArm

#################################################################################################################
HAT_ADDRESS = 0x6F
SERVO_STEP = 5.0             # degrees per keyboard/user-input step

INTERVAL_USERINPUT = 0.03    # seconds
INTERVAL_MOTOR = 0.01        # seconds
INTERVAL_BLINK = 0.35        # seconds
JOYTHRESH = 0.01             # minimum stick movement to register

WINDOW_SIZE = (540, 360)     # pixels
WAYPOINT_FILE = Path(__file__).with_name("controller_direct_waypoints.json")
WAYPOINT_BUTTONS = {
    3: "square",
    0: "x",
    1: "circle",
    2: "triangle",
}
PROGRAM_BUTTONS = {8, 6}     # share/select on common PlayStation/generic mappings
RUN_BUTTONS = {9, 7}         # options/start on common PlayStation/generic mappings
#################################################################################################################


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


def handle_servo_button(button, state, logger):
    """Handle gamepad button functions for direct servo control."""
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


def create_state(arm, logger):
    """Build mutable direct-controller state."""
    base, shoulder, elbow = arm.get_joint_angles()
    gripper = arm.get_gripper_angle()
    return {
        "arm": arm,
        "base": base,
        "shoulder": shoulder,
        "elbow": elbow,
        "gripper": gripper,
        "waypoints": load_waypoints(logger),
        "programming_mode": False,
        "status": "Run mode: press waypoint button to move",
    }


def shutdown_hardware(state):
    """Release servo PWM outputs when the controller exits."""
    arm = state.get("arm")
    if arm is not None:
        arm.deinit()
        state["arm"] = None


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


def update_text(state, screen, font, font_small):
    """Render the direct-controller state."""
    programming_mode = state["programming_mode"]
    blink_on = int(time.time() / INTERVAL_BLINK) % 2 == 0

    screen.fill((255, 255, 255))
    title = font.render("Mode: DIRECT SERVO", True, (0, 0, 0))
    status = font_small.render(state["status"], True, (0, 0, 0))
    screen.blit(title, (10, 10))
    screen.blit(status, (10, 42))

    mode_color = (0, 120, 0)
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
    screen.blit(servo_1, (10, 80))
    screen.blit(servo_2, (10, 102))

    help_lines = [
        "Servo keys:     arrows base/shoulder, W/S elbow, A/D gripper",
        "Servo joystick: axis 0/1/4 joints, triggers or D-pad L/R gripper",
        "Waypoints:     Square/X/Circle/Triangle",
        "Program/run:   left center saves waypoint, right center returns to run",
    ]
    for index, line in enumerate(help_lines):
        rendered = font_small.render(line, True, (0, 0, 0))
        screen.blit(rendered, (10, 150 + index * 24))

    pygame.display.flip()


##############################################################################################################################
# Main
##############################################################################################################################


def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("meArm Direct Controller")
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

    arm = meArm.meArm(address=HAT_ADDRESS, logger=logger)
    state = create_state(arm, logger)

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
                elif event.type == pygame.JOYBUTTONDOWN:
                    if handle_servo_button(event.button, state, logger):
                        redraw = True

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

            if state["programming_mode"] and (current_time - blink_time) > INTERVAL_BLINK:
                redraw = True
                blink_time = current_time

            if redraw:
                update_text(state, screen, font, font_small)

            clock.tick(120)
    finally:
        shutdown_hardware(state)
        pygame.quit()


if __name__ == "__main__":
    main()
