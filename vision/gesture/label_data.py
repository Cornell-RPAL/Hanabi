# converts JSON outputs from OpenPose to numpy array

import numpy as np

poses = ['point_left_hand','point_right_hand']

num_poses = len(poses)

num_no_pose = 79
num_pointL = 60
num_pointR = 58
point_left = np.load('gesture_data/point_left.npy')
point_right = np.load('gesture_data/point_right.npy')
no_pose = np.load('gesture_data/no_pose.npy')

Y = np.zeros((num_no_pose+num_pointL + num_pointR))
Y[:num_no_pose] = 0
Y[num_no_pose:num_no_pose+num_pointL] = 1
Y[num_no_pose+num_pointL:] = 2

# save labels as one_hot_encoded numpy
new_Y = np.asarray(Y, dtype=int)
labels = np.zeros((new_Y.size, num_poses+1))
labels[np.arange(new_Y.size), new_Y] = 1

dataset = np.append(no_pose, point_left, axis = 0)
dataset = np.append(dataset, point_right, axis = 0)
# now, let's shuffle labels and the array, the same way
from sklearn.utils import shuffle
X, Y = shuffle(dataset, labels, random_state = 0)
print(X.shape)
np.save('gesture_data/x', X)
np.save('gesture_data/y', Y)
