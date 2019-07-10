#!/usr/bin/env python
# roslaunch baxter_ikea moveit_init.launch

import rospy
import baxter_interface

rospy.init_node('practice_node')

l_limb = baxter_interface.Limb('left')

angles = limb.joint_angles()
print angles