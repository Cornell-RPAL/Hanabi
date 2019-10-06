#pick up

import rospy
import baxter_interface
import argparse

from motion_consts import NEUTRAL, DISCARD, PREDISCARD, LEFT_END

rospy.init_node('node')

l_limb = baxter_interface.Limb('left')
l_gripper = baxter_interface.Gripper('left')

def discard():
	l_limb.move_to_joint_positions(NEUTRAL)
	l_limb.move_to_joint_positions(PREDISCARD)
	l_limb.move_to_joint_positions(DISCARD)
	l_gripper.open()
	l_limb.move_to_joint_positions(PREDISCARD)
	l_limb.move_to_joint_positions(LEFT_END)
	l_gripper.close(block=True)

discard()