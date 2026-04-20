#!/usr/bin/env python3
#################################################################################################################
# meArm Direct Controller
# Uses keyboard or gamepad to control meArm joint angles directly without inverse kinematics.
#################################################################################################################

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

import logging
import json
from pathlib import Path
import time

import pygame

import meArm

#################################################################################################################
STEP = 5.0                  # degrees per keyboard step
INTERVAL_USERINPUT = 0.03   # seconds
INTERVAL_MOTOR = 0.01       # seconds
JOYTHRESH = 0.01            # minimum stick movement to register
WINDOW_SIZE = (400, 340)    # pixels
WAYPOINT_FILE = Path(__file__).with_name("controller_direct_waypoints.json")
WAYPOINT_BUTTONS = {
    2: "square",
    0: "x",
    1: "circle",
    3: "triangle",
}
PROGRAM_BUTTONS = {8, 6}    # share/select on common PlayStation/generic mappings
RUN_BUTTONS = {9, 7}        # options/start on common PlayStation/generic mappings
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


def check_keys(base, shoulder, elbow, gripper, logger):
    """
    Poll keyboard so one held key can continue driving a joint angle directly.
    """
    keys = pygame.key.get_pressed()
    if keys[pygame.K_RIGHT]:
        base += STEP
        logger.debug("Base increase")
    elif keys[pygame.K_LEFT]:
        base -= STEP
        logger.debug("Base decrease")
    elif keys[pygame.K_UP]:
        shoulder += STEP
        logger.debug("Shoulder increase")
    elif keys[pygame.K_DOWN]:
        shoulder -= STEP
        logger.debug("Shoulder decrease")
    elif keys[pygame.K_w]:
        elbow += STEP
        logger.debug("Elbow increase")
    elif keys[pygame.K_s]:
        elbow -= STEP
        logger.debug("Elbow decrease")
    elif keys[pygame.K_d]:
        gripper += STEP
        logger.debug("Gripper increase")
    elif keys[pygame.K_a]:
        gripper -= STEP
        logger.debug("Gripper decrease")

    return base, shoulder, elbow, gripper


def check_joy_axis(joystick, base, shoulder, elbow, gripper, logger):
    """Handle direct joint-angle input from joystick axes."""
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    joy = False

    base_axis = axis_value(axes, 0)
    shoulder_axis = axis_value(axes, 1)
    elbow_axis = axis_value(axes, 4)
    left_trigger = axis_value(axes, 2, -1.0)
    right_trigger = axis_value(axes, 5, -1.0)

    if abs(base_axis) > JOYTHRESH:
        base += base_axis * STEP
        joy = True
    if abs(shoulder_axis) > JOYTHRESH:
        shoulder -= shoulder_axis * STEP
        joy = True
    if abs(elbow_axis) > JOYTHRESH:
        elbow -= elbow_axis * STEP
        joy = True
    if left_trigger > -1.0 or right_trigger > -1.0:
        gripper += ((left_trigger + 1.0) / 2.0 - (right_trigger + 1.0) / 2.0) * STEP
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


def check_joy_hat(joystick, base, shoulder, elbow, gripper, logger):
    """Handle direct gripper input from the joystick D-pad/hat."""
    if joystick.get_numhats() < 1:
        return base, shoulder, elbow, gripper

    hat_x, _ = joystick.get_hat(0)
    if hat_x > 0:
        gripper += STEP
        logger.debug("Joystick hat gripper increase")
    elif hat_x < 0:
        gripper -= STEP
        logger.debug("Joystick hat gripper decrease")

    return base, shoulder, elbow, gripper


def update_text(base, shoulder, elbow, gripper, programming_mode, status, screen, font, font_small):
    """Report direct joint-angle state in the pygame window."""
    text0 = font.render(f"Base: {base:.0f}", True, (0, 0, 0))
    text1 = font.render(f"Shoulder: {shoulder:.0f}", True, (0, 0, 0))
    text2 = font.render(f"Elbow: {elbow:.0f}", True, (0, 0, 0))
    text3 = font.render(f"Gripper: {gripper:.0f}", True, (0, 0, 0))
    mode = "Waypoint mode: PROGRAM" if programming_mode else "Waypoint mode: RUN"
    text4 = font_small.render(mode, True, (0, 0, 0))
    text5 = font_small.render(status, True, (0, 0, 0))

    help_1 = font_small.render("Keyboard Left/Right: base -/+", True, (0, 0, 0))
    help_2 = font_small.render("Keyboard Up/Down: shoulder +/-", True, (0, 0, 0))
    help_3 = font_small.render("Keyboard W/S: elbow +/-", True, (0, 0, 0))
    help_4 = font_small.render("Keyboard A/D: gripper -/+", True, (0, 0, 0))
    help_5 = font_small.render("Joystick axis 0/1/4: base/shoulder/elbow", True, (0, 0, 0))
    help_6 = font_small.render("Joystick triggers 2/5 or D-pad L/R: gripper", True, (0, 0, 0))
    help_7 = font_small.render("Waypoints: Square/X/Circle/Triangle", True, (0, 0, 0))
    help_8 = font_small.render("Program: left center button, Run: right center", True, (0, 0, 0))

    screen.fill((255, 255, 255))
    screen.blit(text0, (10, 10))
    screen.blit(text1, (10, 40))
    screen.blit(text2, (10, 70))
    screen.blit(text3, (10, 100))
    screen.blit(text4, (10, 130))
    screen.blit(text5, (10, 150))

    screen.blit(help_1, (10, 180))
    screen.blit(help_2, (10, 200))
    screen.blit(help_3, (10, 220))
    screen.blit(help_4, (10, 240))
    screen.blit(help_5, (10, 260))
    screen.blit(help_6, (10, 280))
    screen.blit(help_7, (10, 300))
    screen.blit(help_8, (10, 320))
    pygame.display.flip()


##############################################################################################################################
# Main
##############################################################################################################################


def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    arm = meArm.meArm(address=0x6F, logger=logger)
    base, shoulder, elbow = arm.get_joint_angles()
    gripper = arm.get_gripper_angle()
    waypoints = load_waypoints(logger)
    programming_mode = False
    status = "Press waypoint button to move, program button to save"

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("meArm Direct Controller")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 36)
    font_small = pygame.font.SysFont(None, 16)

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        logger.info("Controller detected: %s", joystick.get_name())
        joy = True
    except pygame.error:
        logger.warning("No controller detected.")
        joy = False

    motor_time = time.time()
    check_userinput_time = time.time()

    update_text(base, shoulder, elbow, gripper, programming_mode, status, screen, font, font_small)

    running = True
    while running:
        current_time = time.time()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button in PROGRAM_BUTTONS:
                    programming_mode = True
                    status = "Programming: press waypoint button to save"
                    update_text(base, shoulder, elbow, gripper, programming_mode, status, screen, font, font_small)
                elif event.button in RUN_BUTTONS:
                    programming_mode = False
                    status = "Run mode: press waypoint button to move"
                    update_text(base, shoulder, elbow, gripper, programming_mode, status, screen, font, font_small)
                else:
                    base, shoulder, elbow, gripper, waypoint_status = handle_waypoint_button(
                        event.button,
                        base,
                        shoulder,
                        elbow,
                        gripper,
                        waypoints,
                        programming_mode,
                        logger,
                    )
                    if waypoint_status is not None:
                        status = waypoint_status
                        update_text(base, shoulder, elbow, gripper, programming_mode, status, screen, font, font_small)

        if (current_time - check_userinput_time) > INTERVAL_USERINPUT:
            base, shoulder, elbow, gripper = check_keys(base, shoulder, elbow, gripper, logger)
            if joy:
                base, shoulder, elbow, gripper = check_joy_axis(
                    joystick,
                    base,
                    shoulder,
                    elbow,
                    gripper,
                    logger,
                )
                base, shoulder, elbow, gripper = check_joy_hat(
                    joystick,
                    base,
                    shoulder,
                    elbow,
                    gripper,
                    logger,
                )
            check_userinput_time = current_time

        if (current_time - motor_time) > INTERVAL_MOTOR:
            if (base, shoulder, elbow) != arm.get_joint_angles():
                base, shoulder, elbow = arm.set_joint_angles(base, shoulder, elbow)
                update_text(base, shoulder, elbow, gripper, programming_mode, status, screen, font, font_small)

            if gripper != arm.get_gripper_angle():
                gripper = arm.set_gripper_angle(gripper)
                update_text(base, shoulder, elbow, gripper, programming_mode, status, screen, font, font_small)

            motor_time = current_time

        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    main()
