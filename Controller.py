#!/usr/bin/env python3
#################################################################################################################
# meArm Controller
# Uses keyboard or gamepad to control meArm
#################################################################################################################

# ##############################################################################
# 
# The MIT License (MIT)
# 
# Copyright (c) 225 Urs Utzinger
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

import pygame
import meArm
import time
import logging

#################################################################################################################
STEP = 5.                   ## in mm
INTERVAL_USERINPUT = 0.03   ## in seconds
INTERVAL_MOTOR = 0.01       ## in seconds
JOYTHRESH = 0.01             ## threshold to register joystick activity
WINDOW_SIZE = (400, 300)    ## in pixels
#################################################################################################################

def clamp(val, mn, mx):
    return max(mn, min(mx, val))

def checkKeys(x,y,z,finger,logger):
    """
    Poll keyboard, 
    this allows pushing multiple keys at once
    """
    keys = pygame.key.get_pressed()
    if   keys[pygame.K_UP]: ## Moves meArm FORWARD when "up arrow" key is pressed
        y = y + STEP
        logger.debug("Up")
    elif keys[pygame.K_DOWN]: ## Moves meArm BACKWARD when "down arrow" key is pressed
        y = y - STEP
        logger.debug("Down")
    elif keys[pygame.K_RIGHT]: ## Moves meArm RIGHT when "right arrow" key is pressed
        x = x + STEP
        logger.debug("Right")
    elif keys[pygame.K_LEFT]: ## Moves meArm LEFT when "left arrow" key is pressed
        x = x - STEP
        logger.debug("Left")
    elif keys[pygame.K_w]: ## Moves meArm UP when "w" key is pressed
        z = z + STEP
        logger.debug("w")
    elif keys[pygame.K_s]: ## Moves meArm DOWN when "s" key is pressed
        z = z - STEP
        logger.debug("s")
    elif keys[pygame.K_o]: ## FULLY opens gripper
        finger = 100.
        logger.debug("o")
    elif keys[pygame.K_l]: ## FULLY closes gripper
        finger = 0.
        logger.debug("l")
    elif keys[pygame.K_p]: ## PARTIALLY opens gripper, to  50%
        finger = 50.
        logger.debug("p")
    elif keys[pygame.K_q]: ## PARTIALLY opens grippeer
        finger = finger + STEP
        logger.debug("q")
    elif keys[pygame.K_a]: ## PARTIALLY closes gripper
        finger = finger - STEP
        logger.debug("a")
    else:
        pass

    return x,y,z,finger

def checkJoyAxis(joystick, x,y,z,finger,logger):
    '''Handle joystick position'''
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    joy = False
    if axes[0] > JOYTHRESH or axes[0] < -JOYTHRESH:
        x += axes[0]*STEP
        joy = True
    if axes[1] > JOYTHRESH or axes[1] < -JOYTHRESH:
        y -= axes[1]*STEP
        joy = True
    if axes[4] > JOYTHRESH or axes[4] < -JOYTHRESH:
        z -= axes[4]*STEP
        joy = True
    if (axes[2] > -1.0) or (axes[5] > -1.0):
        gr = (axes[2]+1)/2 - (axes[5]+1)/2
        finger +=gr*STEP
        joy = True
    
    if joy:
        logger.debug(f"Joystick x:{x:.0f} y:{y:.0f} z:{z:.0f} gripper:{finger:.0f}")
    
    return x,y,z,finger


def on_joyhat(button,x,y,z,finger,logger):
    """
    Top left rocker switch
    Forward, Backward, Left and Right Buttons
    """
    if   button == ( 0, 1): # Forward
        y = y + 20.
    elif button == ( 0,-1): # Backward
        y = y - 20.
    elif button == ( 1, 0): # Right
        x = x + 20.
    elif button == (-1, 0): # Left
        x = x - 20.
    else:
        pass

    logger.debug(f"Hat: x:{x:.0f} y:{y:.0f} z:{z:.0f} gripper:{finger:.0f}")
    return x,y,z,finger

def on_joybutton(button,x,y,z,finger,logger):
    '''Handle joystick button events '''
    reverse_attack_time = 0.
    reverse_defense_time =0.
    if     button == 0: # cross, defense
        x = 0. 
        y = 85.  
        z = 30.
        logger.debug("defense")
    elif   button == 1: # ring, attack right
        x += 75.
        y = 200.
        logger.debug("attack right")
    elif   button == 2: # triangle attack middle
        y = 200.
        logger.debug("attack middle")
    elif   button == 3: # square: attack forward left
        x -= 75.
        y = 200.
        logger.debug("attack left")
    elif   button == 4: # left front top
        finger = 100.
        logger.debug("open gripper")
    elif   button == 5: # right front top
        finger = 0.
        logger.debug("close gripper")
    elif   button == 6: # left front bottom
        pass # is coupled to axis
    elif   button == 7: # right front bottom
        pass # is coupled to axis
    elif   button == 8: # top front left: attack
        y += 40.
        reverse_attack_time = time.time() + 0.2
        logger.debug(f"attack sequence: {reverse_attack_time:.0f}")
    elif   button == 9: # top fropnt right: defense
        y -= 40.
        reverse_defense_time = time.time() + 0.5
        logger.debug(f"defense sequence: {reverse_defense_time:.0f}")
    elif  button == 10: # home
        x = 0.   ## original x
        y = 125. ## original y
        z = 75.  ## original z
        logger.debug("home")
    else:
        logger.debug(f"Unassigned Button: {button}")

    return x,y,z,finger, reverse_attack_time, reverse_defense_time

def updateText(x,y,z,finger,screen, font, font_small):
    '''Report system status in game window'''
    
    text0 = font.render(f"X: {x:.0f}", True, (0, 0, 0))
    text1 = font.render(f"Y: {y:.0f}", True, (0, 0, 0))
    text2 = font.render(f"Z: {z:.0f}", True, (0, 0, 0))
    text3 = font.render(f"Gripper: {finger:.0f}", True, (0, 0, 0))

    help_1 = font_small.render("Keyboard: Arrows/w-s: to move", True, (0, 0, 0))
    help_2 = font_small.render("Keyboard: o/l/p/q/a for gripper", True, (0, 0, 0))
    help_3 = font_small.render("Joystick Left: u/d/r/l: Forward/Backward/Right/Left", True, (0, 0, 0))
    help_4 = font_small.render("Joystick Right: u/d: Up/Down", True, (0, 0, 0))
    help_5 = font_small.render("Joybutton: Cross: defense, O/Tri/Square: attack right, middle, left", True, (0, 0, 0))
    help_6 = font_small.render("Joybutton Front: Left/Right: gripper Open/Close", True, (0, 0, 0))
    help_7 = font_small.render("Joybutton Top Middle: Left/Right: Attack/Defend", True, (0, 0, 0))
    help_8 = font_small.render("Joyhat: Left/Right/Up/Down: Left/Right/Forward/Backward", True, (0, 0, 0))
    
    # Blit text to screen
    screen.fill((255, 255, 255)) 
    screen.blit(text0,  (10, 10))
    screen.blit(text1,  (10, 40))
    screen.blit(text2, (10, 70))
    screen.blit(text3, (10, 100))

    screen.blit(help_1, (10, 130))
    screen.blit(help_2, (10, 150))
    screen.blit(help_3, (10, 170))
    screen.blit(help_4, (10, 190))
    screen.blit(help_5, (10, 210))
    screen.blit(help_6, (10, 230))
    screen.blit(help_7, (10, 250))
    screen.blit(help_8, (10, 270))
    pygame.display.flip()


##############################################################################################################################
# Main
##############################################################################################################################

def main():

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    # - meArm initialize ----------------------------------
    arm = meArm.meArm(address=0x6F, logger=logger) # create the driver

    # apply start values
    x,y,z = arm.get_position()
    finger = arm.get_finger()

    # - Py Game -------------------------------
    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("meArm Controller")
    clock = pygame.time.Clock()
    font=pygame.font.SysFont(None, 36)
    font_small=pygame.font.SysFont(None, 16)

    try:
        # Initialize the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        logger.info("Controller detected: ", joystick.get_name())
        joy = True
    except pygame.error:
        logger.warning("No controller detected.")
        joy = False

    # - Loop -----------------------------------
    motor_time      = time.time() # joystick command
    check_userinput_time = time.time() # keyboard command
    reverse_attack_time = 0
    reverse_defense_time = 0

    updateText(x,y,z,finger,screen,font,font_small)
    
    running = True
    while running:
        current_time = time.time()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # Exit when window is closed
                running = False

            # - Game Pad Buttons -------------------
            elif event.type == pygame.JOYBUTTONDOWN:
                # Handle each button press
                logger.debug("Button pressed: ", event.button)
                (x, y, z, finger, reverse_attack_time, reverse_defense_time) = (
                    on_joybutton(event.button,x,y,z,finger,logger)
                )

            # - Game Pad Hat Buttons ---------------
            elif event.type == pygame.JOYHATMOTION:
                x, y, z, finger = on_joyhat(event.value,x,y,z,finger,logger)
                logger.debug("Hat pressed: ", event.value)
                    
            else:
                pass
                logger.debug("Unknown event:", event)

        if (current_time - check_userinput_time) > INTERVAL_USERINPUT:
            # - Keyboard Keys ----------------------
            x,y,z,finger = checkKeys(x,y,z,finger,logger)
            # - Game Pad Joysticks -----------------
            if joy:
                x, y, z, finger = checkJoyAxis(joystick, x,y,z,finger,logger)
            check_userinput_time = current_time

        if (current_time - motor_time) > INTERVAL_MOTOR:
            # Move arm if we have new position
            if (x, y, z) != arm.get_position():
                arm.move_to(x, y, z)
                x, y, z = arm.get_position()
                updateText(x,y,z,finger,screen,font,font_small)
                
            # Adjust gripper if we have new location
            if finger != arm.get_finger():
                arm.partial_grip(finger)
                finger = arm.get_finger()
                updateText(x,y,z,finger,screen,font,font_small)

            motor_time = current_time


        # Deal with timed movement sequences
        
        #   attack move
        if (reverse_attack_time < current_time) and (reverse_attack_time != 0.):
            y = y - 40.
            reverse_attack_time = 0.
            updateText(x,y,z,finger,screen, font, font_small)

        #   defense move
        if (reverse_defense_time < current_time) and (reverse_defense_time != 0.):
            y = y + 40.
            reverse_defense_time = 0.
            updateText(x,y,z,finger,screen, font, font_small)

        clock.tick(60)
    
    pygame.quit()

if __name__ == '__main__':
    main()
