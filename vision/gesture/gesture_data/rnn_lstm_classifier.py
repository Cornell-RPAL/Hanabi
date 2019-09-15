import keras
from keras.models import Sequential, load_model
from keras.optimizers import Adam
from keras.layers import Dense, Dropout, LSTM, Flatten, BatchNormalization, TimeDistributed, Activation
from sklearn.model_selection import train_test_split
import tensorflow as tf
import numpy as np

X = np.load("gesture_data/x.npy")
y = np.load("gesture_data/y.npy")

x_train, x_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state = 4)

x_train = keras.utils.normalize(x_train, axis = 1)
x_test = keras.utils.normalize(x_test, axis = 1)

epochs = 200
batch_size = 67
_dropout = 0.5
_activation='relu'
_optimizer='adam'
input_shape = x_train.shape[1], x_train.shape[2]
model = Sequential()
model.add(LSTM(128, input_shape=input_shape, return_sequences = True))
model.add(Dropout(_dropout))                             
model.add(BatchNormalization())

model.add(LSTM(128, input_shape=input_shape, return_sequences = True))
model.add(Dropout(_dropout))                             
model.add(BatchNormalization())

model.add(LSTM(64, input_shape=input_shape, return_sequences = True))
model.add(Dropout(_dropout))                             
model.add(BatchNormalization())

model.add(LSTM(32, input_shape=input_shape, return_sequences = True))
model.add(Dropout(_dropout))                             
model.add(BatchNormalization())

model.add(TimeDistributed(Dense(3))) 

model.add(Flatten())
model.add(Dense(y_train.shape[1], activation = tf.nn.softmax))
model.add(Activation(_activation))

model.compile(optimizer=_optimizer,
               loss='categorical_crossentropy',
               metrics=['accuracy'])
model.fit(x_train, y_train, epochs=epochs,batch_size=batch_size)
model.summary()
loss, acc = model.evaluate(x_test, y_test,
                                batch_size=batch_size)

print('Loss: {:.3}'.format(loss))
print('Acc: {:.3}'.format(acc))

model.save('gesture_data/pointing.h5')