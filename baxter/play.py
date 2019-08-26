#!/usr/bin/env python

import rospy
import baxter_interface
import argparse

from motion_consts import NEUTRAL, POSITIONS, LEFT_END

rospy.init_node('node')

l_limb = baxter_interface.Limb('left')
l_gripper = baxter_interface.Gripper('left')

def pick_up(i):
	l_limb.move_to_joint_positions(NEUTRAL)
	l_limb.move_to_joint_positions(POSITIONS['place_start'][i])
	l_gripper.open()
	l_limb.move_to_joint_positions(POSITIONS['place_end'][i])
	l_gripper.close(block=True)
	l_limb.move_to_joint_positions(LEFT_END)


parser = argparse.ArgumentParser(description="play card parser")
parser.add_argument("i", nargs='?', default="check_string_for_empty")
args = parser.parse_args()

if args.i == 'check_string_for_empty':
    print 'No index given, please specify the index of the card you want played.'
elif int(args.i) in range(0,5):
    pick_up(int(args.i))
else:
    print 'Give a valid integer in range 0-4 inclusive to specify the index!'