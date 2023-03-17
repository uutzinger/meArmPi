# -*- coding: utf-8 -*-
#
#  DemoIK.py - York Hack Space May 2014
#  Simple demo of meArm library to walk through some points defined in Cartesian coordinates

import meArm

arm = meArm.meArm()
arm.begin(0,0x70) # address of motor controller

arm.gotoPoint(   0, 150,   50)
arm.openGripper()
arm.closeGripper()

# Move Along Z
# ======================
# 0..100
arm.gotoPoint(0,150,100)
# Mark location
arm.gotoPoint(0,150, 50)
# Mark location
arm.gotoPoint(0,150, 0)
# Mark location
# My Results for Z
#  50 to 100 is 60
#   0 to  50 is 55

# Move Along X
# ======================
# -50..50
arm.gotoPoint(  0,150,50)
# Mark location
arm.gotoPoint( 50,150,50)
# Mark location
arm.gotoPoint(-50,150,50)
# Mark location
# Results for X
# -50 to 0 is 55
#   0 to 50 is 50

# Move Along Y
# ====================
# My Range is 110..210
# You might be able to reach 100..200
# In Y direction closest location might be 90, I ran into stop at 110 because my wings are not mounted ideally
arm.gotoPoint(  0,110,50)
# Mark location
arm.gotoPoint(  0,150,50)
# Mark location
arm.gotoPoint(- 0,200,50)
# Mark location
# Results for Y
# 110 to 150 is 40
# 150 to 200 is 50

# From these numbers you can see that we have about +/- 5mm accuracy.
# The range is 200mm or less, depending on the height (location in z) of the claw

# Moving arm to the corners of a box
#
arm.gotoPoint( 50,110,0)
arm.gotoPoint(-50,110,0)
arm.gotoPoint(-50,110,100)
arm.gotoPoint( 50,110,100)
arm.gotoPoint( 50,180,100)
arm.gotoPoint(-50,180,100)
arm.gotoPoint(-50,180,  0)
arm.gotoPoint( 50,180,  0)
arm.gotoPoint( 50,110,  0)

arm.gotoPoint(  0,150, 50)
