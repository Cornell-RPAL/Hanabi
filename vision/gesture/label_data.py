# converts JSON outputs from OpenPose to numpy array

import numpy as np

poses = ['point_left_hand','point_right_hand']

point_left = np.load('gesture_data/point_left.npy')

print(point_left[0])
print(point_left[0].shape)
