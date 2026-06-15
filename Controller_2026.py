#!/usr/bin/env python3
#################################################################################################################
# meArm Cartesian Controller
# Uses keyboard or gamepad to control the meArm end effector with inverse kinematics.
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
CARTESIAN_STEP = 5.0        # millimeters per keyboard/user-input step
GRIPPER_STEP = 5.0          # percentage points per keyboard/user-input step

INTERVAL_USERINPUT = 0.03   # seconds
INTERVAL_MOTOR = 0.01       # seconds
INTERVAL_BLINK = 0.35       # seconds
JOYTHRESH = 0.01            # minimum stick movement to register

WINDOW_SIZE = (540, 380)    # pixels
WAYPOINT_FILE = Path(__file__).with_name("controller_waypoints.json")
WAYPOINT_BUTTONS = {
    3: "square",
    0: "x",
    1: "circle",
    2: "triangle",
}
PROGRAM_BUTTONS = {8, 6}    # share/select on common PlayStation/generic mappings
RUN_BUTTONS = {9, 7}        # options/start on common PlayStation/generic mappings
HOME_BUTTONS = {10}
OPEN_BUTTONS = {4}
CLOSE_BUTTONS = {5}
#################################################################################################################


def axis_value(axes, index, default=0.0):
    """Return an axis value if the controller exposes it."""
    if index < len(axes):
        return axes[index]
    return default


def cartesian_distance(point_a, point_b):
    """Return Cartesian distance between two x/y/z tuples."""
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    dz = point_a[2] - point_b[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def waypoint_position(x, y, z, finger):
    """Return a serializable Cartesian waypoint."""
    return {
        "x": x,
        "y": y,
        "z": z,
        "finger": finger,
    }


def load_waypoints(logger):
    """Load Cartesian controller waypoints from JSON."""
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
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "z": float(position["z"]),
                    "finger": float(position["finger"]),
                }
            except (KeyError, TypeError, ValueError):
                logger.warning("Ignoring invalid waypoint %s in %s", name, WAYPOINT_FILE)

    return waypoints


def save_waypoints(waypoints, logger):
    """Save Cartesian controller waypoints to JSON."""
    try:
        with WAYPOINT_FILE.open("w") as f:
            json.dump(waypoints, f, indent=4)
            f.write("\n")
    except OSError as exc:
        logger.warning("Could not save %s: %s", WAYPOINT_FILE, exc)


def handle_waypoint_button(button, state, logger):
    """Store or recall a Cartesian waypoint from a face-button press."""
    name = WAYPOINT_BUTTONS.get(button)
    if name is None:
        return False

    if state["programming_mode"]:
        state["waypoints"][name] = waypoint_position(
            state["x"],
            state["y"],
            state["z"],
            state["finger"],
        )
        save_waypoints(state["waypoints"], logger)
        state["status"] = f"Saved {name} waypoint"
        logger.info(state["status"])
        return True

    position = state["waypoints"].get(name)
    if position is None:
        state["status"] = f"No {name} waypoint saved"
        logger.info(state["status"])
        return True

    state["x"] = position["x"]
    state["y"] = position["y"]
    state["z"] = position["z"]
    state["finger"] = position["finger"]
    state["status"] = f"Loaded {name} waypoint"
    logger.info(state["status"])
    return True


def check_cartesian_keys(x, y, z, finger, logger):
    """Poll keyboard for Cartesian inverse-kinematics control."""
    keys = pygame.key.get_pressed()
    if keys[pygame.K_RIGHT]:
        x += CARTESIAN_STEP
        logger.debug("X increase")
    elif keys[pygame.K_LEFT]:
        x -= CARTESIAN_STEP
        logger.debug("X decrease")
    elif keys[pygame.K_UP]:
        y += CARTESIAN_STEP
        logger.debug("Y increase")
    elif keys[pygame.K_DOWN]:
        y -= CARTESIAN_STEP
        logger.debug("Y decrease")
    elif keys[pygame.K_w]:
        z += CARTESIAN_STEP
        logger.debug("Z increase")
    elif keys[pygame.K_s]:
        z -= CARTESIAN_STEP
        logger.debug("Z decrease")
    elif keys[pygame.K_o]:
        finger = 100.0
        logger.debug("Gripper open")
    elif keys[pygame.K_l]:
        finger = 0.0
        logger.debug("Gripper close")
    elif keys[pygame.K_p]:
        finger = 50.0
        logger.debug("Gripper half-open")
    elif keys[pygame.K_q]:
        finger += GRIPPER_STEP
        logger.debug("Gripper increase")
    elif keys[pygame.K_a]:
        finger -= GRIPPER_STEP
        logger.debug("Gripper decrease")

    return x, y, z, finger


def check_cartesian_joy_axis(joystick, x, y, z, finger, logger):
    """Handle Cartesian input from joystick axes."""
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    joy = False

    x_axis = axis_value(axes, 0)
    y_axis = axis_value(axes, 1)
    z_axis = axis_value(axes, 4)
    left_trigger = axis_value(axes, 2, -1.0)
    right_trigger = axis_value(axes, 5, -1.0)

    if abs(x_axis) > JOYTHRESH:
        x += x_axis * CARTESIAN_STEP
        joy = True
    if abs(y_axis) > JOYTHRESH:
        y -= y_axis * CARTESIAN_STEP
        joy = True
    if abs(z_axis) > JOYTHRESH:
        z -= z_axis * CARTESIAN_STEP
        joy = True
    if left_trigger > -1.0 or right_trigger > -1.0:
        finger += ((left_trigger + 1.0) / 2.0 - (right_trigger + 1.0) / 2.0) * GRIPPER_STEP
        joy = True

    if joy:
        logger.debug("Joystick x:%.0f y:%.0f z:%.0f gripper:%.0f", x, y, z, finger)

    return x, y, z, finger


def check_cartesian_joy_hat(joystick, x, y, z, finger, logger):
    """Handle Cartesian X/Y input from the joystick D-pad/hat."""
    if joystick.get_numhats() < 1:
        return x, y, z, finger

    hat_x, hat_y = joystick.get_hat(0)
    if hat_x > 0:
        x += CARTESIAN_STEP
        logger.debug("Joystick hat X increase")
    elif hat_x < 0:
        x -= CARTESIAN_STEP
        logger.debug("Joystick hat X decrease")

    if hat_y > 0:
        y += CARTESIAN_STEP
        logger.debug("Joystick hat Y increase")
    elif hat_y < 0:
        y -= CARTESIAN_STEP
        logger.debug("Joystick hat Y decrease")

    return x, y, z, finger


def handle_cartesian_button(button, state, logger):
    """Handle gamepad button functions for Cartesian control."""
    if button in PROGRAM_BUTTONS:
        state["programming_mode"] = True
        state["status"] = "Programming: press waypoint button to save"
        return True
    if button in RUN_BUTTONS:
        state["programming_mode"] = False
        state["status"] = "Run mode: press waypoint button to move"
        return True
    if button in OPEN_BUTTONS:
        state["finger"] = 100.0
        state["status"] = "Gripper open"
        return True
    if button in CLOSE_BUTTONS:
        state["finger"] = 0.0
        state["status"] = "Gripper closed"
        return True
    if button in HOME_BUTTONS:
        state["x"] = 0.0
        state["y"] = 150.0
        state["z"] = 100.0
        state["status"] = "Home waypoint loaded"
        return True

    return handle_waypoint_button(button, state, logger)


def create_state(arm, logger):
    """Build mutable Cartesian-controller state."""
    x, y, z = arm.get_position()
    return {
        "arm": arm,
        "x": x,
        "y": y,
        "z": z,
        "finger": arm.get_finger(),
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


def service_cartesian_mode(state, joystick, joy, current_time, check_userinput_time, motor_time, logger):
    """Poll and service Cartesian inverse-kinematics controls."""
    changed = False
    if (current_time - check_userinput_time) > INTERVAL_USERINPUT:
        state["x"], state["y"], state["z"], state["finger"] = check_cartesian_keys(
            state["x"],
            state["y"],
            state["z"],
            state["finger"],
            logger,
        )
        if joy:
            state["x"], state["y"], state["z"], state["finger"] = check_cartesian_joy_axis(
                joystick,
                state["x"],
                state["y"],
                state["z"],
                state["finger"],
                logger,
            )
            state["x"], state["y"], state["z"], state["finger"] = check_cartesian_joy_hat(
                joystick,
                state["x"],
                state["y"],
                state["z"],
                state["finger"],
                logger,
            )
        check_userinput_time = current_time

    arm = state["arm"]
    if arm is not None and (current_time - motor_time) > INTERVAL_MOTOR:
        if (state["x"], state["y"], state["z"]) != arm.get_position():
            requested = (state["x"], state["y"], state["z"])
            if arm.move_to(*requested):
                actual = arm.get_position()
                if cartesian_distance(requested, actual) > 1.0:
                    state["status"] = "Moved to nearest pose allowed by joint/servo limits"
                else:
                    state["status"] = "Moved to requested Cartesian position"
            else:
                state["status"] = "Requested Cartesian position is out of IK range"
            state["x"], state["y"], state["z"] = arm.get_position()
            changed = True

        if state["finger"] != arm.get_finger():
            arm.partial_grip(state["finger"])
            state["finger"] = arm.get_finger()
            changed = True

        motor_time = current_time

    return check_userinput_time, motor_time, changed


def update_text(state, screen, font, font_small):
    """Render the Cartesian-controller state."""
    programming_mode = state["programming_mode"]
    blink_on = int(time.time() / INTERVAL_BLINK) % 2 == 0

    screen.fill((255, 255, 255))
    title = font.render("Mode: CARTESIAN IK", True, (0, 0, 0))
    status = font_small.render(state["status"], True, (0, 0, 0))
    screen.blit(title, (10, 10))
    screen.blit(status, (10, 42))

    mode_color = (0, 90, 190)
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

    cartesian_1 = font_small.render(
        f"Cartesian x/y/z: {state['x']:.0f} / {state['y']:.0f} / {state['z']:.0f} mm",
        True,
        (0, 0, 0),
    )
    cartesian_2 = font_small.render(f"Gripper open: {state['finger']:.0f}%", True, (0, 0, 0))
    screen.blit(cartesian_1, (10, 80))
    screen.blit(cartesian_2, (10, 102))

    help_lines = [
        "Cartesian keys: arrows X/Y, W/S Z, O/L/P/Q/A gripper",
        "Joystick:       axis 0/1/4 X/Y/Z, triggers gripper, D-pad X/Y",
        "Waypoints:      Square/X/Circle/Triangle use direct move_to commands",
        "Program/run:    left center saves waypoint, right center returns to run",
        "Home:           home button returns to x=0, y=150, z=100",
        "Limits:         displayed pose is the actual pose after IK, joint, and servo clamps",
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
    pygame.display.set_caption("meArm Cartesian Controller 2026")
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
                    if handle_cartesian_button(event.button, state, logger):
                        redraw = True

            check_userinput_time, motor_time, changed = service_cartesian_mode(
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
