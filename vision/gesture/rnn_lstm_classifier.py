import itertools
import numpy as np

from keras.models import Sequential
from keras.layers import Dense, Flatten, Dropout, TimeDistributed, Activation
from keras.layers import LSTM
from sklearn.cross_validation import train_test_split
from sklearn.metrics import confusion_matrix
# from kerasify import export_model
from keras.models import model_from_json

def samples_to_3D_array(_vector_dim, _vectors_per_sample, _X):
    '''
    Keras LSTM model require 3-Dimensional Tensors.
    Function convert samples to 3D tensors !
    '''
    X_len = len(_X)
    result_array = []
    for sample in range (0,X_len):
        sample_array = []
        for vector_idx in range (0, _vectors_per_sample):
            start = vector_idx * _vector_dim
            end = start + _vector_dim
            sample_array.append(_X[sample][start:end])
        result_array.append(sample_array)

    return np.asarray(result_array)

def main():
    '''
    main routine to train and generate keras classifier model
    '''
    # Model Parameters and Paths

    timesteps = 5
    epochs = 1000
    batch_size = 32
    _dropout = 0.1
    _activation='relu'
    _optimizer='adam'
  #  class_names = ["fist","pinch","wave","victory","stop","thumbsup"]
    X_vector_dim = 1 # number of features or columns (hand)
    X_path = "gesture_data/xtrain.npy"
    Y_path = "gesture_data/ytrain.npy"
    # model_path = '../../../train_data/hand/hand.model'
    # json_model_path = '../../../train_data/hand/hand_model.json'
    # model_weights_path = "../../../train_data/hand/hand_model.h5"

    # Load Keypoints Samples and Labels
    X = np.load(X_path, dtype="float")
    Y = np.load(Y_path)

    y_vector_dim = y.shape[1] # number of features or columns

    X_vectors_per_sample = timesteps # number of vectors per sample

    # Keras LSTM model require 3-Dimensional Tensors. Convert samples to 3D tensors
    X_3D = samples_to_3D_array(X_vector_dim, X_vectors_per_sample, X)

    # Perform test-train split
    X_train, X_test, y_train, y_test = train_test_split(X_3D, Y, test_size=0.33, random_state=42)

    input_shape = (X_train.shape[1], X_train.shape[2]) # store input_shape

    print ("Model Parameters:")
    print ("input_shape     : ", input_shape)
    print ("X_vector_dim    : ", X_vector_dim)
    print ("y_vector_dim    : ", y_vector_dim)

    # Build Keras TimeDistributed(Dense) (many-to-many case) LSTM model
    print ("Build Keras Timedistributed-LSTM Model...")
    model = Sequential()
    model.add(TimeDistributed(Dense(X_vector_dim, activation=_activation), input_shape=input_shape))
    model.add(Dropout(_dropout))
    model.add(TimeDistributed(Dense(X_vector_dim*2, activation=_activation))) #(5, 80)
    model.add(Dropout(_dropout))
    model.add(TimeDistributed(Dense(X_vector_dim, activation=_activation))) #(5, 40)
    model.add(Dropout(_dropout))
    model.add(TimeDistributed(Dense(X_vector_dim/2, activation=_activation))) #(5, 20)
    model.add(Dropout(_dropout))
    model.add(TimeDistributed(Dense(X_vector_dim/4, activation=_activation))) #(5, 10)
    model.add(Dropout(_dropout))
    model.add(LSTM(X_vector_dim/4, dropout=_dropout, recurrent_dropout=_dropout))
    model.add(Dense(y_vector_dim,activation='softmax'))
    model.compile(loss='categorical_crossentropy', optimizer=_optimizer, metrics=['accuracy'])
    model.summary()

    # Fit model
    print('Training...')
    model.fit(X_train, y_train,
              batch_size=batch_size,
              epochs=epochs,
              validation_data=(X_test, y_test))

    # Evaluate Model and Predict Classes
    print('Testing...')
    score, accuracy = model.evaluate(X_test, y_test,
                                     batch_size=batch_size)

    print('Test score: {:.3}'.format(score))
    print('Test accuracy: {:.3}'.format(accuracy))

    # Export model
    # export_model(model,model_path)

    # # serialize model to JSON
    # model_json = model.to_json()
    # with open(json_model_path, "w") as json_file:
    #     json_file.write(json_model_path)
    # # serialize weights to HDF5
    # model.save_weights(model_weights_path)
    # print("Saved model to disk")



    # from keras.models import Sequential
    # from keras.layers import Dense, Dropout, Activation, Flatten
    # from keras.optimizers import SGD
    
    # model = Sequential()
    # model.add(Dense(128, activation='relu', input_shape=(60,)))
    # model.add(Dropout(0.5))
    # model.add(Dense(128, activation='relu'))
    # model.add(Dropout(0.5))
    # model.add(Dense(y1.shape[1], activation='softmax'))
    # model.compile(optimizer='Adam',
    #               loss='categorical_crossentropy',
    #               metrics=['accuracy'])
    # model.fit(X1, y1, epochs=2000,batch_size=21)
    
    model.save('gesture_data/pointing.h5') # save our model as h5


if __name__ == "__main__":
    main()