
import rospy
import baxter_interface

rospy.init_node('practice_node')

l_limb = baxter_interface.Limb('left')
r_limb = baxter_interface.Limb('right')

l_gripper = baxter_interface.Gripper('left')

down = {'left_w0': -3.0322965224524916, 'left_w1': -0.9180875015494201, 'left_w2': -1.8296555847503273, 'left_e0': 0.11389807350049197, 'left_e1': 0.9234564343070191, 'left_s0': -0.7175195135334023, 'left_s1': -0.3021942152134265}
up = {'left_w0': -3.046869339937403, 'left_w1': -1.3706118339756173, 'left_w2': -1.615665264840312, 'left_e0': 0.14764565083397108, 'left_e1': 0.7386117493668267, 'left_s0': -0.7516505860638527, 'left_s1': -0.46479617872928036}

l_limb.move_to_joint_positions(up)
l_limb.move_to_joint_positions(down)
l_gripper.open(block=True)
l_limb.move_to_joint_positions(up)
