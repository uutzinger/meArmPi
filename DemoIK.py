#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  DemoIK.py - York Hack Space May 2014
#  Simple demo of meArm library to walk through some points defined in Cartesian coordinates

import meArm


arm = meArm2.meArm()
arm.begin(0,0x60) # block address of motor controller

arm.openGripper()
arm.closeGripper()
arm.openGripper()
arm.closeGripper()
arm.openGripper()

arm.gotoPoint(   0, 150,   0)
arm.gotoPoint(   0, 150,  50)
arm.gotoPoint(   0, 150, 100)
arm.gotoPoint(   0, 150,  50)
arm.gotoPoint(   0, 200,  50)
arm.gotoPoint(   0, 100,  50)
arm.gotoPoint(   0,  80,  50)
arm.gotoPoint(   0, 150,  50)
arm.gotoPoint(-100, 150,  50)
arm.gotoPoint( 100, 150,  50)
arm.gotoPoint(   0, 150,  50)




