##
## Sebastian (Sebo) Diaz, 04/09/2021, BME210 Spring 2021
## Mark Bosset
##

## Type "pip3 install pynput" on UnixShell in RaspberryPi to install keyboard library

import pygame
import meArm
import time

x = 0            ## original x coordinate position, change as needed
y = 125          ## original y coordinate position, change as needed
z = 75           ## original z coordinate position, change as needed
finger = 50
increment = 10

pygame.init()
pygame.joystick.init()


arm = meArm.meArm() # takes inserted data from meArm.py aka calibration data
arm.begin(0,0x70) # block address of motor controller, initializes home position (0, 150, 100) and opens Gripper

arm.goDirectlyTo(x,y,z)
x = arm.x
y = arm.y
z = arm.z
arm.paropenGripper(finger)
finger = arm.finger

def on_press(key):
    global x,y,z, increment, finger
    
    if   key == pygame.K_UP: ## Moves meArm FORWARD when "w" key is pressed
        y_n = y + increment
        if not arm.goDirectlyTo(x,y_n,z): print ("out of range")
    elif key == pygame.K_DOWN: ## Moves meArm BACKWARD when "s" key is pressed
        y_n = y - increment
        if not arm.goDirectlyTo(x,y_n,z): print ("out of range")
    elif key == pygame.K_RIGHT: ## Moves meArm RIGHT when "d" key is pressed
        x_n = x + increment
        if not arm.goDirectlyTo(x_n,y,z): print ("out of range")
    elif key == pygame.K_LEFT: ## Moves meArm LEFT when "a" key is pressed
        x_n = x - increment
        if not arm.goDirectlyTo(x_n,y,z): print ("out of range")
    elif key == pygame.K_u: ## Moves meArm UP when "u" key is pressed
        z_n = z + increment
        if not arm.goDirectlyTo(x,y,z_n): print ("out of range")
    elif key == pygame.K_j: ## Moves meArm DOWN when "j" key is pressed
        z_n = z - increment
        if not arm.goDirectlyTo(x,y,z_n): print ("out of range")
    elif key == pygame.K_o: ## FULLY opens gripper
        if not arm.openGripper(): print ("out of range")
    elif key == pygame.K_l: ## FULLY closes gripper
        if not arm.closeGripper(): print ("out of range")
    elif key == pygame.K_k: ## PARIALLY opens gripper, to percentage (inputed, default is 50% but this can be modified) of full open state when "k" key is pressed
        if not arm.paropenGripper(50): print ("out of range")
    elif key == pygame.K_p: ## PARIALLY opens gripper, to percentage (inputed, default is 50% but this can be modified) of full open state when ";" key is pressed
        finger_n = finger + increment
        if not arm.parcloseGripper(finger_n): print ("out of range")
    elif key == pygame.K_SEMICOLON: ## PARIALLY opens gripper, to percentage (inputed, default is 50% but this can be modified) of full open state when ";" key is pressed
        finger_n = finger - increment
        if not arm.parcloseGripper(finger_n): print ("out of range")
    else:
        pass

    x = arm.x
    y = arm.y
    z = arm.z
    finger = arm.finger

def on_joyaxis(axis, value):
    # for joystick events
    global x,y,z, increment, finger

    if      axis == 0: # left right
        x_n = x + value*increment
        if not arm.goDirectlyTo(x_n,y,z): print ("out of range")
    elif    axis == 1: ## forward backward
        y_n = y - value*increment
        if not arm.goDirectlyTo(x,y_n,z): print ("out of range")
    elif    axis == 4: ## upo, down
        z_n = z - value*increment
        if not arm.goDirectlyTo(x,y,z_n): print ("out of range")
    elif    axis == 3: ## open close
        finger_n = finger + value*increment
        if not arm.paropenGripper(finger_n): print ("out of range")

    elif    axis == 2: ## attack
        y_attack = y + 40
        if not arm.goDirectlyTo(x,y_attack,z): print ("out of range")
        time.sleep(0.1)
        if not arm.goDirectlyTo(x,y,z): print ("out of range")
        
    elif    axis == 5: ## defense
        y_defense = y - 40
        if not arm.goDirectlyTo(x,y_defense,z): print ("out of range")
        time.sleep(0.1)
        if not arm.goDirectlyTo(x,y,z): print ("out of range")

    else:
        pass

    x = arm.x
    y = arm.y
    z =  arm.z
    finger = arm.finger


def on_joybutton(button):
    # for joystick events
    global x,y,z, increment, finger

    if     button == 0: # X
        if not arm.goDirectlyTo(x,y,z): print ("out of range")
    elif   button == 1: # O
        x_n = 0   ## original x
        y_n = 125 ## original y
        z_n = 75  ## original z
        if not arm.goDirectlyTo(x_n,y_n,z_n): print("out of range")

    elif   button == 2: # triangle
        if not arm.goDirectlyTo(x,y,z): print ("out of range")
    elif   button == 3: # squre
        if not arm.goDirectlyTo(x,y,z): print ("out of range") 
    elif   button == 4: # left
        if not arm.openGripper(): print ("out of range")
    elif   button == 5: # right
        if not arm.closeGripper(): print ("out of range")
    else:
        pass

    x = arm.x
    y = arm.y
    z = arm.z
    finger = arm.finger

try:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Controller detected: ", joystick.get_name())
except pygame.error:
    print("No joystick detected.")
    quit()

screen = pygame.display.set_mode((400,300))
pygame.display.set_caption("Keyboard input")
clock = pygame.time.Clock()

previous_axis = 0
action_time = time.time()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit()

        if event.type == pygame.JOYBUTTONDOWN:
            print("Button pressed: ", event.button)
            on_joybutton(event.button)
            
        if event.type == pygame.JOYAXISMOTION:
            print("Axis moved: ", event.axis, event.value)
            if event.axis == 2 or event.axis == 5: 
                # We want to run attack and defense only once when the buttons are touched but they provide several events
                if not (previous_axis == event.axis):
                    on_joyaxis(event.axis, event.value)
                    previous_axis = event.axis
                    action_time = time.time()
                else:
                    # but we also want to unlock the block after some timeout.
                    if (time.time() - action_time) > 0.3:
                        previous_axis = 0

            else:
                on_joyaxis(event.axis, event.value)
                previous_axis = event.axis
                action_time = time.time()

        if event.type == pygame.KEYDOWN:
            print("Key pressed: ", event.key)
            on_press(event.key)


    clock.tick(100)

