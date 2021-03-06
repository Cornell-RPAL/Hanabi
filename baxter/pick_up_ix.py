import rospy
import baxter_interface
import argparse
import time

from motion_consts import NEUTRAL, POSITIONS
from log import log

rospy.init_node('node')

l_limb = baxter_interface.Limb('left')
l_gripper = baxter_interface.Gripper('left')

def pick_up(i):
	l_limb.move_to_joint_positions(NEUTRAL)
	l_limb.move_to_joint_positions(POSITIONS['above_holder'][i])
	l_gripper.open()
        time.sleep(2)
	l_limb.move_to_joint_positions(POSITIONS['holder'][i])
	l_gripper.close(block=True)
	l_limb.move_to_joint_positions(POSITIONS['above_holder'][i])
	l_limb.move_to_joint_positions(NEUTRAL)

parser = argparse.ArgumentParser(description="pick up card parser")
parser.add_argument("i", nargs='?', default="check_string_for_empty")
args = parser.parse_args()

if args.i == 'check_string_for_empty':
    log('No index given, please specify the index of the card you want picked up.')
elif int(args.i) in range(0,5):
    pick_up(int(args.i))
else:
    log('Give a valid integer in range 0-4 inclusive to specify the index!')
