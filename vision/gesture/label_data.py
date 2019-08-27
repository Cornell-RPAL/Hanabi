# converts JSON outputs from OpenPose to numpy array

import numpy as np

poses = ['point_left_hand','point_right_hand']

num_poses = len(poses)
num_pointL = 48
num_pointR = 35
point_left = np.load('gesture_data/point_left.npy')
point_right = np.load('gesture_data/point_right.npy')

Y = np.zeros((num_pointL + num_pointR))
Y[:num_pointL] = 1
Y[num_pointL:] = 2

# save labels as one_hot_encoded numpy
new_Y = np.asarray(Y, dtype=int)
labels = np.zeros((new_Y.size, num_poses+1))
labels[np.arange(new_Y.size), new_Y] = 1

dataset = np.append(point_left, point_right, axis=0)

# now, let's shuffle labels and the array, the same way
from sklearn.utils import shuffle
X, Y = shuffle(dataset, labels, random_state = 0)

X[:,:,0] = X[:,:,0] / 960
X[:,:,1] = X[:,:,1] / 720
X = X[:,:,1:]
print(X.shape)
X = X.reshape(len(X), 60)

np.save('gesture_data/xtrain', X)
np.save('gesture_data/ytrain', Y)
