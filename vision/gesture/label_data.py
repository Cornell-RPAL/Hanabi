# converts JSON outputs from OpenPose to numpy array

import numpy as np

poses = ['point_left_hand','point_right_hand']

num_poses = len(poses)
num_pointL = 60
num_pointR = 44

Y = np.zeros((num_pointL + num_pointR))
Y[:num_pointL] = 1
Y[num_pointL:] = 2

# save labels as one_hot_encoded numpy
new_Y = np.asarray(Y, dtype=int)
y_one_hot = np.zeros((new_Y.size, num_poses+1))
y_one_hot[np.arange(new_Y.size), new_Y] = 1
