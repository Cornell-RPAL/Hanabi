#!/usr/bin/env python

import rospy
import baxter_interface
import argparse
import time

from motion_consts import NEUTRAL_R, POINT
from log import log

rospy.init_node('node')

l_limb = baxter_interface.Limb('right')

def point(i_s):
	for i in i_s:
		l_limb.move_to_joint_positions(POINT[i])
		time.sleep(2)
	l_limb.move_to_joint_positions(NEUTRAL_R)

parser = argparse.ArgumentParser(description="point at card parser")
parser.add_argument('--nargs-int-type', nargs='+', type=int)
	

for _, values in parser.parse_args()._get_kwargs():
	for v in values:
		if v not in range(0,5):
			log('all arguments must be integers from 0-4 inclusive')
	point(values)
