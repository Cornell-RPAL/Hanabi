#!/usr/bin/env python

import rospy
import baxter_interface

rospy.init_node('practice_node')

l_limb = baxter_interface.Limb('left')
r_limb = baxter_interface.Limb('right')

l_gripper = baxter_interface.Gripper('left')

clutch_card = {'left_w0': -3.046869339937403, 'left_w1': -1.0956457777471567, 'left_w2': -1.6444274046131635, 'left_e0': 0.0038349519697135344, 'left_e1': 0.9184709967463914, 'left_s0': -0.5660389107297177, 'left_s1': -0.31945149907713744}
higher = {'left_w0': -3.044951863952546, 'left_w1': -1.445393397385031, 'left_w2': -1.6616846884768743, 'left_e0': 0.05062136600021865, 'left_e1': 0.8367865197914932, 'left_s0': -0.5602864827751474, 'left_s1': -0.5748593002600588}
above_board = {'left_w0': -3.0319130272555204, 'left_w1': -0.5836796897904, 'left_w2': -1.8960002538263714, 'left_e0': 0.05138835639416136, 'left_e1': 0.8195292359277823, 'left_s0': -0.9809807138527221, 'left_s1': -0.25310683000109324}

l_limb.move_to_joint_positions(higher)
l_gripper.open()
l_limb.move_to_joint_positions(clutch_card)
l_gripper.close(block=True)
l_limb.move_to_joint_positions(higher)
l_limb.move_to_joint_positions(above_board)
l_gripper.open(block=True)
l_limb.move_to_joint_positions(higher)

#l_limb.move_to_joint_positions(wave_2)

		