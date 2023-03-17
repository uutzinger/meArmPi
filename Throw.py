from pynput.keyboard import Listener
import meArm
import time

arm = meArm.meArm() # takes inserted data from meArm.py aka calibration data
arm.begin(0,0x70) #

xs = -100 ## original x coordinate position, change as needed
ys = 100 ## original y coordinate position, change as needed
zs = 0 ## original z coordinate position, change as needed

arm.gotoPoint(xs,ys,zs) 
arm.closeGripper()

def on_press(key):
    global xs,ys,zs, xe,ye,ze
    var = str(format(key))
    semi = '\';\''
    
    if var == semi: ## PARIALLY opens gripper, to percentage (inputed, default is 50% but this can be modified) of full open state when ";" key is pressed
        xe = 0 ## original x coordinate position, change as needed
        ye = 200 ## original y coordinate position, change as needed
        ze = 45 ## original z coordinate position, change as needed
        arm.goDirectlyTo(xe,ye,ze) 
        time.sleep(5.)
        arm.gotoPoint(xs,ys,zs) 
        
    pass

def on_release(key):
    pass

with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
