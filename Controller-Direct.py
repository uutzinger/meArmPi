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
import time

import pygame

import meArm

#################################################################################################################
STEP = 5.0                  # degrees per keyboard step
INTERVAL_USERINPUT = 0.03   # seconds
INTERVAL_MOTOR = 0.01       # seconds
JOYTHRESH = 0.01            # minimum stick movement to register
WINDOW_SIZE = (400, 300)    # pixels
#################################################################################################################


def axis_value(axes, index, default=0.0):
    """Return an axis value if the controller exposes it."""
    if index < len(axes):
        return axes[index]
    return default


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


def update_text(base, shoulder, elbow, gripper, screen, font, font_small):
    """Report direct joint-angle state in the pygame window."""
    text0 = font.render(f"Base: {base:.0f}", True, (0, 0, 0))
    text1 = font.render(f"Shoulder: {shoulder:.0f}", True, (0, 0, 0))
    text2 = font.render(f"Elbow: {elbow:.0f}", True, (0, 0, 0))
    text3 = font.render(f"Gripper: {gripper:.0f}", True, (0, 0, 0))

    help_1 = font_small.render("Keyboard Left/Right: base -/+", True, (0, 0, 0))
    help_2 = font_small.render("Keyboard Up/Down: shoulder +/-", True, (0, 0, 0))
    help_3 = font_small.render("Keyboard W/S: elbow +/-", True, (0, 0, 0))
    help_4 = font_small.render("Keyboard A/D: gripper -/+", True, (0, 0, 0))
    help_5 = font_small.render("Joystick axis 0/1/4: base/shoulder/elbow", True, (0, 0, 0))
    help_6 = font_small.render("Joystick triggers 2/5: gripper open/close", True, (0, 0, 0))

    screen.fill((255, 255, 255))
    screen.blit(text0, (10, 10))
    screen.blit(text1, (10, 40))
    screen.blit(text2, (10, 70))
    screen.blit(text3, (10, 100))

    screen.blit(help_1, (10, 140))
    screen.blit(help_2, (10, 160))
    screen.blit(help_3, (10, 180))
    screen.blit(help_4, (10, 200))
    screen.blit(help_5, (10, 230))
    screen.blit(help_6, (10, 250))
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

    update_text(base, shoulder, elbow, gripper, screen, font, font_small)

    running = True
    while running:
        current_time = time.time()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

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
            check_userinput_time = current_time

        if (current_time - motor_time) > INTERVAL_MOTOR:
            if (base, shoulder, elbow) != arm.get_joint_angles():
                base, shoulder, elbow = arm.set_joint_angles(base, shoulder, elbow)
                update_text(base, shoulder, elbow, gripper, screen, font, font_small)

            if gripper != arm.get_gripper_angle():
                gripper = arm.set_gripper_angle(gripper)
                update_text(base, shoulder, elbow, gripper, screen, font, font_small)

            motor_time = current_time

        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    main()
