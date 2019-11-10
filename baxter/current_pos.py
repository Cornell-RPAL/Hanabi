#!/usr/bin/env python

import rospy
import baxter_interface
import argparse

from motion_consts import NEUTRAL, POSITIONS
from log import log

rospy.init_node('node')

l_limb = baxter_interface.Limb('right')
log(l_limb.joint_angles())
