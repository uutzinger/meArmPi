# meArm.py - York Hack Space May 2014
# A motion control library for Phenoptix meArm using Adafruit 16-channel PWM servo driver

# Updates
# partial gripper open
# returns if move actional compled successfully
# keeps current  x,y,z,gripper as property
# doe snot move beyond max and min angle

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
    value=0
    x=0
    y=0
    z=0
    finger = 0
    def __init__(self,
            sweepMinBase     =  96, sweepMaxBase     = 504, angleMinBase     =   1.66, angleMaxBase     = -1.43,
            sweepMinShoulder = 129, sweepMaxShoulder = 451, angleMinShoulder =   2.44, angleMaxShoulder =  0.0,
            sweepMinElbow    = 440, sweepMaxElbow    = 232, angleMinElbow    =   0.44, angleMaxElbow    = -1.13,
            sweepMinGripper  = 402, sweepMaxGripper  = 172, angleMinGripper  =   0.00, angleMaxGripper  =  1.75):
        """Constructor for meArm - can use as default arm=meArm(), or supply calibration data for servos."""
        self.servoInfo = {}
        self.servoInfo["base"]     = self.setupServo(sweepMinBase,     sweepMaxBase,     angleMinBase,     angleMaxBase)
        self.servoInfo["shoulder"] = self.setupServo(sweepMinShoulder, sweepMaxShoulder, angleMinShoulder, angleMaxShoulder)
        self.servoInfo["elbow"]    = self.setupServo(sweepMinElbow,    sweepMaxElbow,    angleMinElbow,    angleMaxElbow)
        self.servoInfo["gripper"]  = self.setupServo(sweepMinGripper, sweepMaxGripper,   angleMinGripper,  angleMaxGripper)
       
    ## Adafruit servo driver has four 'blocks' of four servo connectors, 0, 1, 2 or 3.
    def begin(self, block = 0, address = 0x6f):
        """Call begin() before any other meArm calls.  Optional parameters to select a different block of servo connectors or different I2C address."""
        self.mh = Adafruit_MotorHAT(address, i2c_bus=1) # Address of Adafruit PWM servo driver
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
        rec["gain"]   = gain
        rec["zero"]   = zero
        rec["n_min"]  = n_min
        rec["n_max"]  = n_max
        rec["a_max"]  = max(a_min, a_max)
        rec["a_min"]  = min(a_min, a_max)
        print(rec)
        return rec

    def checkAngle(self, servo, angle):
        """Check whether angle is between max and min of calibration angles"""
        if self.servoInfo[servo]["a_min"] <= angle <= self.servoInfo[servo]["a_max"]: 
            return True
        else:                             
            return False
    
    def angle2pwm(self, servo, angle):
        """Work out pulse length to use to achieve a given requested angle taking into account stored calibration data"""
        ret = int(self.servoInfo[servo]["zero"] + self.servoInfo[servo]["gain"] * angle)
        return ret
       
    def goDirectlyTo(self, x, y, z):
        """Set servo angles so as to place the gripper at a given Cartesian point as quickly as possible, without caring what path it takes to get there"""
        angles = [0,0,0]
        if kinematics.solve(x, y, z, angles):

            radBase     = angles[0]
            radShoulder = angles[1]
            radElbow    = angles[2]

            if self.checkAngle("base", radBase) and self.checkAngle("shoulder", radShoulder) and self.checkAngle("elbow", radElbow) :

                #print("Angle       %s" % ([radBase*180.0/3.141, radShoulder*180.0/3.141, radElbow*180.0/3.141]))
                #print("PWM         %s" % ([self.angle2pwm("base", radBase),self.angle2pwm("shoulder", radShoulder), self.angle2pwm("elbow", radElbow)]))
                #print("Unsolve XYZ %s" % ([kinematics.unsolve(radBase, radShoulder, radElbow)]))
                self.pwm.setPWM(self.base,     0, self.angle2pwm("base",     radBase))
                self.pwm.setPWM(self.shoulder, 0, self.angle2pwm("shoulder", radShoulder))
                self.pwm.setPWM(self.elbow,    0, self.angle2pwm("elbow",    radElbow))
                self.x = x
                self.y = y
                self.z = z
                print("Going to %s" % ([x,y,z]))
                return True
            else:
                print("out of range")
                return False
        else:
            print("out of range")
            return False

    def gotoPoint(self, x, y, z):
        """Travel in a straight line from current position to a requested position"""
        x0 = self.x
        y0 = self.y
        z0 = self.z
        dist = kinematics.distance(x0, y0, z0, x, y, z)
        step = 10
        i = 0
        while i < dist:
            if self.goDirectlyTo(x0 + (x - x0) * i / dist, y0 + (y - y0) * i / dist, z0 + (z - z0) * i / dist):
                time.sleep(0.05)
                i += step
            else:
                break
        return self.goDirectlyTo(x, y, z)


    def openGripper(self):
        """Open the gripper, dropping whatever is being carried"""
        self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", self.servoInfo["gripper"]["a_max"]))
        self.finger = 100

    def closeGripper(self):
        """Close the gripper, grabbing onto anything that might be there"""
        self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", self.servoInfo["gripper"]["a_min"]))
        self.finger = 0

    def paropenGripper(self, value):
        """Partially opens the gripper, dropping whatever is being carried"""
        if value > 100: value = 100
        if value <   0: value =   0
        self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", (1/100)*value*(self.servoInfo["gripper"]["a_max"]-self.servoInfo["gripper"]["a_min"])))
        self.finger = value

    def isReachable(self, x, y, z):
        """Returns True if the point is (theoretically) reachable by the gripper"""
        radBase = 0
        radShoulder = 0
        radElbow = 0
        if kinematics.solve(x, y, z, radBase, radShoulder, radElbow):
            if self.checkAngle("base", radBase) and self.checkAngle("shoulder", radShoulder) and self.checkAngle("elbow", radElbow) :
                return True
            else:
                return False
        else:
            return False

    
    def getPos(self):
        """Returns the current position of the gripper"""
        return [self.x, self.y, self.z]
