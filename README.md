meArmPi
=======

Movement control library in Python for Phenoptix meArm on Raspberry Pi via Adafruit `adafruit_motor` and `adafruit_pca9685` 

The meArm has four servos one for the

    - gripper (SG90), 
    - base (SG90), 
    - shoulder (S3003),
    - elbow (S3003). 
    
This library provies ability to position the gripper in a Cartesian (x, y, z) coordinate system.

It solves the angles required to set the servos in order to meet a given position.

Coordinates are (approximately) measured in mm from the base rotation centre. Initial 'home' position is at (0, 100, 50), i.e. 100mm forward of the base and 50mm off the ground.

Various other versions of this library exist:
* [Arduino](https://github.com/yorkhackspace/meArm)
* [Arduino with Adafruit PWM driver board](https://github.com/RorschachUK/meArm_Adafruit)
* [Beaglebone Black](https://github.com/RorschachUK/meArmBBB)

[![meArm moving with Inverse Kinematics](http://img.youtube.com/vi/HbxhVs3UmuE/0.jpg)](http://www.youtube.com/watch?v=HbxhVs3UmuE)

This library also contains Demo programs:

    - `Controller.py` allowing keyboard and jostick input using pygame
    - `Zero.py` to calibrate the motors

Wiring
------

This uses an Adafruit 16-channel PWM servo driver board to connect the servos to the Raspberry Pi.  Use the first block of four servo connectors, and connect yellow wire to the top, brown wire to the bottom.
* Servo 0: meArm rotating base
* Servo 1: meArm shoulder (right hand side servo)
* Servo 2: meArm elbow (left hand side servo)
* Servo 3: meArm gripper


Usage
-----

```
import meArm

def main():
    arm = meArm.meArm(address=0x40)
	
    while True:
        arm.open_gripper()
        arm.close_gripper()
        arm.open_gripper()
        arm.close_gripper()
        arm.open_gripper()
        
        #Go up and left to grab something
        arm.move_to(-80,100,140) 
        arm.close_gripper()
        #Go down, forward and right to drop it
        arm.move_linear(70,200,10)
        arm.openGripper()
        #Back to start position
        arm.move_to(0,100,50)
    return 0

if __name__ == '__main__':
	main()
```

Installation
------------
* Clone this repository to your local machine
* Run with sudo, i.e. 'sudo python DemoIK.py'

Class methods of meArm object
-----------------------------
* begin(address=0x6F) - determines i2c address to use for the motor controller
* open_gripper() - opens the gripper, letting go of anything it was holding
* close_gripper() - closes the gripper, perhaps grabbing and holding something as it does so
* partial_grip(prct) - opens gripper to prct per cent
* move_liner(x, y, z, step) - move in a straight line from the current point to the requested position
* move_to(x, y, z) - set the servo angles to immediately go to the requested point without caring what path the arm swings through to get there - faster but less predictable
* get_position() - current [x, y, z] coordinates
* get_finger - current gripper open status in per cent
