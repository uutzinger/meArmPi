# meArm.py - York Hack Space May 2014
# A motion control library for Phenoptix meArm using Adafruit 16-channel PWM servo driver

# from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import kinematics
import time
from math import pi
import numpy

# To properly calibrate identify SHOULDER, ELBOW, BASE, GRIPPER
# Then move the motor with a demo porgram to the disired location and measure the angle
# This gives you PWM and angle. You will need two measurements for each motor to compute gain and zero.
# Do NOT just use the code below, as each constructed arm will require its own calibration.

# Shoulder might be max min reverted

class meArm():
    value = 0
    def __init__(self,
            sweepMinBase     = 111, sweepMaxBase     = 492, angleMinBase     =  1.4, angleMaxBase     = -1.52,
            sweepMinShoulder = 143, sweepMaxShoulder = 379, angleMinShoulder =  1.8,  angleMaxShoulder =  0.05,
            sweepMinElbow    = 410, sweepMaxElbow    = 205, angleMinElbow    =  0.44, angleMaxElbow    = -1.05,
            sweepMinGripper  = 482, sweepMaxGripper  = 205, angleMinGripper  =  -0.79, angleMaxGripper  =  0.79):
        """Constructor for meArm - can use as default arm=meArm(), or supply calibration data for servos."""
        self.servoInfo = {}
        self.servoInfo["base"]     = self.setupServo(sweepMinBase, sweepMaxBase, angleMinBase, angleMaxBase)
        self.servoInfo["shoulder"] = self.setupServo(sweepMinShoulder, sweepMaxShoulder, angleMinShoulder, angleMaxShoulder)
        self.servoInfo["elbow"]    = self.setupServo(sweepMinElbow, sweepMaxElbow, angleMinElbow, angleMaxElbow)
        self.servoInfo["gripper"]  = self.setupServo(sweepMinGripper, sweepMaxGripper, angleMinGripper, angleMaxGripper)
       
    ## Adafruit servo driver has four 'blocks' of four servo connectors, 0, 1, 2 or 3.
    def begin(self, block = 0, address = 0x6f):
        """Call begin() before any other meArm calls.  Optional parameters to select a different block of servo connectors or different I2C address."""
        self.mh = Adafruit_MotorHAT(address) # Address of Adafruit PWM servo driver
        self.pwm = self.mh._pwm
        self.base = block * 4
        self.shoulder = block * 4 + 1
        self.elbow = block * 4 + 14
        self.gripper = block * 4 + 15
        self.pwm.setPWMFreq(50)
        self.openGripper()
        self.goDirectlyTo(0, 150, 100)
       
    def setupServo(self, n_min, n_max, a_min, a_max):
        """Calculate servo calibration record to place in self.servoInfo"""
        rec = {}
        n_range = n_max - n_min
        a_range = a_max - a_min
        if a_range == 0: return
        gain = n_range / a_range
        zero = n_min - gain * a_min
        rec["gain"] = gain
        rec["zero"] = zero
        rec["min"] = n_min
        rec["max"] = n_max
        print(rec)
        return rec
    
    def angle2pwm(self, servo, angle):
        """Work out pulse length to use to achieve a given requested angle taking into account stored calibration data"""
        ret = int(self.servoInfo[servo]["zero"] + self.servoInfo[servo]["gain"] * angle)
        return ret
       
    def goDirectlyTo(self, x, y, z):
        """Set servo angles so as to place the gripper at a given Cartesian point as quickly as possible, without caring what path it takes to get there"""
        angles = [0,0,0]
        if kinematics.solve(x, y, z, angles):
            radBase = angles[0]
            radShoulder = angles[1]
            radElbow = angles[2]
            print("Angle       %s" % ([radBase*180.0/3.141, radShoulder*180.0/3.141, radElbow*180.0/3.141]))
            print("PWM         %s" % ([self.angle2pwm("base", radBase),self.angle2pwm("shoulder", radShoulder), self.angle2pwm("elbow", radElbow)]))
            print("Unsolve XYZ %s" % ([kinematics.unsolve(radBase, radShoulder, radElbow)]))
            self.pwm.setPWM(self.base, 0, self.angle2pwm("base", radBase))
            self.pwm.setPWM(self.shoulder, 0, self.angle2pwm("shoulder", radShoulder))
            self.pwm.setPWM(self.elbow, 0, self.angle2pwm("elbow", radElbow))
            self.x = x
            self.y = y
            self.z = z
            print("Going to %s" % ([x,y,z]))

    def gotoPoint(self, x, y, z):
        """Travel in a straight line from current position to a requested position"""
        x0 = self.x
        y0 = self.y
        z0 = self.z
        dist = kinematics.distance(x0, y0, z0, x, y, z)
        step = 10
        i = 0
        while i < dist:
            self.goDirectlyTo(x0 + (x - x0) * i / dist, y0 + (y - y0) * i / dist, z0 + (z - z0) * i / dist)
            time.sleep(0.05)
            i += step
        self.goDirectlyTo(x, y, z)
        time.sleep(0.05)
    def openGripper(self):
        """Open the gripper, dropping whatever is being carried"""
        self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", pi/4.0))
        time.sleep(0.3)
    def closeGripper(self):
        """Close the gripper, grabbing onto anything that might be there"""
        self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", -pi/4.0))
        time.sleep(0.3)
    def paropenGripper(self, value):
        """Partially opens the gripper, dropping whatever is being carried"""
        self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", (1/100)*value*(pi/4.0)))
        time.sleep(0.3)
    def parcloseGripper(self, value):
        """Partially closes the gripper, grabbing onto anything that might be there"""
        self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", (1/100)*value*(-pi/4.0)))
        time.sleep(0.3)
    def isReachable(self, x, y, z):
        """Returns True if the point is (theoretically) reachable by the gripper"""
        radBase = 0
        radShoulder = 0
        radElbow = 0
        return kinematics.solve(x, y, z, radBase, radShoulder, radElbow)
    
    def getPos(self):
        """Returns the current position of the gripper"""
        return [self.x, self.y, self.z]
