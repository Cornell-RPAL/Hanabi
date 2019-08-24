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
X1, y1 = shuffle(dataset, labels)

X1[:,:,0] = X1[:,:,0] / 960
X1[:,:,1] = X1[:,:,1] / 720
X1 = X1[:,:,1:]
print(X1.shape)
X1 = X1.reshape(len(X1), 60)

from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.optimizers import SGD

model = Sequential()
model.add(Dense(128, activation='relu', input_shape=(60,)))
model.add(Dropout(0.5))
model.add(Dense(128, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(y1.shape[1], activation='softmax'))
model.compile(optimizer='Adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])
model.fit(X1, y1, epochs=2000,batch_size=21)

model.save('gesture_data/pointing.h5') # save our model as h5
