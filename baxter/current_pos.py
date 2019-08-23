#!/usr/bin/env python

import rospy
import baxter_interface
import argparse

from motion_consts import NEUTRAL, POSITIONS

rospy.init_node('node')

l_limb = baxter_interface.Limb('right')
print(l_limb.joint_angles())