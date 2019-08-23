#!/usr/bin/env python

import rospy
import baxter_interface
import argparse

from motion_consts import NEUTRAL_R, POINT

rospy.init_node('node')

l_limb = baxter_interface.Limb('right')

def point(i):
	l_limb.move_to_joint_positions(POINT[i])
	l_limb.move_to_joint_positions(NEUTRAL_R)

parser = argparse.ArgumentParser(description="point at card parser")
parser.add_argument("i", nargs='?', default="check_string_for_empty")
args = parser.parse_args()

if args.i == 'check_string_for_empty':
    print 'No index given, please specify the index of the card you want pointed at.'
elif int(args.i) in range(0,5):
    point(int(args.i))
else:
    print 'Give a valid integer in range 0-4 inclusive to specify the index!'