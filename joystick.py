##
## Sebastian (Sebo) Diaz, 04/09/2021, BME210 Spring 2021
## Mark Bosset
##

## Type "pip3 install pynput" on UnixShell in RaspberryPi to install keyboard library

from pynput.keyboard import Listener
import meArm

arm = meArm.meArm() # takes inserted data from meArm.py aka calibration data
arm.begin(0,0x70) # block address of motor controller, initializes home position (0, 150, 100) and opens Gripper

x = 0 ## original x coordinate position, change as needed
y = 125 ## original y coordinate position, change as needed
z = 75 ## original z coordinate position, change as needed

arm.gotoPoint(x,y,z) ## HOME POSITION, can be edited to whatever you please
increment = input("Please enter a incremental value: ") ## Asks user for an incremental value 
increment = int(increment) ## converst input into an integer

def on_press(key):
    global x,y,z, increment
    var = str(format(key))
    w = '\'w\''
    s = '\'s\''
    d = '\'d\''
    a = '\'a\''
    i = '\'i\''
    k = '\'k\''
    o = '\'o\''
    l = '\'l\''
    m = '\'m\''
    u = '\'u\''
    j = '\'j\''
    semi = '\';\''
    
    if var == w: ## Moves meArm FORWARD when "w" key is pressed
        y = y + increment
        arm.gotoPoint(x,y,z)
    elif var == s: ## Moves meArm BACKWARD when "s" key is pressed
        y = y - increment
        arm.gotoPoint(x,y,z)
    elif var == d: ## Moves meArm RIGHT when "d" key is pressed
        x = x + increment
        arm.gotoPoint(x,y,z)
    elif var == a: ## Moves meArm LEFT when "a" key is pressed
        x = x - increment
        arm.gotoPoint(x,y,z)
    elif var == u: ## Moves meArm UP when "u" key is pressed
        z = z + increment
        arm.gotoPoint(x,y,z)
    elif var == j: ## Moves meArm DOWN when "j" key is pressed
        z = z - increment
        arm.gotoPoint(x,y,z)
    elif var == o: ## FULLY opens gripper
        arm.openGripper()
    elif var == l: ## FULLY closes gripper
        arm.closeGripper()
    elif var == k: ## PARIALLY opens gripper, to percentage (inputed, default is 50% but this can be modified) of full open state when "k" key is pressed
        arm.paropenGripper(50)
    elif var == semi: ## PARIALLY opens gripper, to percentage (inputed, default is 50% but this can be modified) of full open state when ";" key is pressed
        arm.parcloseGripper(50)
    pass

def on_release(key):
    pass

with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()