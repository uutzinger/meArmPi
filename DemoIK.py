#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  DemoIK.py - York Hack Space May 2014

import meArm

def main():
	arm = meArm.meArm()
	arm.begin()
	
	while True:
		arm.openGripper()
		arm.closeGripper()
		arm.openGripper()
		arm.closeGripper()
		arm.openGripper()
	
		arm.gotoPoint(0, 150, 50)
		arm.gotoPoint(0, 150, 0)
		arm.gotoPoint(0, 150, 150)
		arm.gotoPoint(0, 150, 50)
		arm.gotoPoint(-100, 150, 50)
		arm.gotoPoint(100, 150, 50)
		arm.gotoPoint(0, 150, 50)
		arm.gotoPoint(0, 100, 50)
		
	return 0

if __name__ == '__main__':
	main()
